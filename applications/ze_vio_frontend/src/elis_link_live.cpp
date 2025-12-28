// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <gflags/gflags.h>
#include <ze/common/logging.hpp>

#include <ze/common/types.hpp>
#include <ze/vio_frontend/frontend_api.hpp>
#include <ze/vio_frontend/frontend_base.hpp>

#include <dvs_msgs/Event.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#if defined(HAVE_METAVISION_SDK) && __has_include(<metavision/sdk/base/events/event_cd.h>) && __has_include(<metavision/sdk/driver/camera.h>)
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/driver/camera.h>
#define HAVE_METAVISION_SDK_IMPL 1
#else
#undef HAVE_METAVISION_SDK_IMPL
#endif

DEFINE_string(elis_live_serial, "",
              "Optional camera serial ID. If empty, uses the first available camera.");
DEFINE_uint64(elis_live_slice_dt_us, 10000,
              "If >0, slice the live stream into fixed time windows (microseconds) "
              "instead of fixed event counts. Set to 0 to use --elis_live_events_per_frame.");
DEFINE_uint64(elis_live_events_per_frame, 50000,
              "Number of events per frontend frame when --elis_live_slice_dt_us=0.");
DEFINE_int32(elis_live_max_frames, 0,
             "Maximum number of frames to process (0 = run until interrupted).");
DEFINE_string(elis_live_out_tum, "",
              "Optional output path for a TUM trajectory log (t[s] tx ty tz qx qy qz qw).");
DEFINE_int32(elis_live_queue_size, 2,
             "Max number of event slices queued between the camera callback and frontend processing. "
             "If exceeded, slices are dropped (oldest first).");

namespace {

struct TumWriter
{
  explicit TumWriter(const std::string& path)
  {
    if (!path.empty())
    {
      out_.open(path);
      if (!out_.is_open())
      {
        std::cerr << "[Live] Failed to open TUM output: " << path << "\n";
      }
    }
  }

  void write(const int64_t t_ns,
             const Eigen::Quaterniond& q,
             const Eigen::Vector3d& p)
  {
    if (!out_.is_open())
    {
      return;
    }
    const double t_s = static_cast<double>(t_ns) * 1e-9;
    out_ << std::fixed << std::setprecision(9)
         << t_s << " "
         << p.x() << " " << p.y() << " " << p.z() << " "
         << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    out_.flush();
  }

  std::ofstream out_;
};

void process_event_slice(ze::FrontendBase& frontend,
                         ze::EventArrayPtr events_ptr)
{
  if (!events_ptr || events_ptr->empty())
  {
    return;
  }

  const int64_t t_start_ns = events_ptr->front().ts.toNSec();
  const int64_t t_end_ns = events_ptr->back().ts.toNSec();
  const int64_t t_mid_ns = (t_start_ns + t_end_ns) / 2;

  // FrontendBase expects the next IMU batch to start at the last stamp of the
  // previous batch (DEBUG_CHECK_EQ in addImuMeasurementsBetweenKeyframes()).
  // In events-only mode we don't have real IMU, so we stitch synthetic stamps
  // together to satisfy that contract.
  static int64_t last_imu_end_ns = -1;
  const int64_t t_imu_start_ns = (last_imu_end_ns >= 0) ? last_imu_end_ns : t_start_ns;
  last_imu_end_ns = t_end_ns;

  ze::ImuStamps imu_stamps(3);
  imu_stamps << t_imu_start_ns, t_mid_ns, t_end_ns;

  ze::ImuAccGyrContainer imu_accgyr(6, 3);
  imu_accgyr.setZero();

  std::vector<ze::ImuStamps> imu_stamps_vec{imu_stamps};
  std::vector<ze::ImuAccGyrContainer> imu_accgyr_vec{imu_accgyr};

  ze::StampedEventArray stamped_events{t_end_ns, std::move(events_ptr)};
  frontend.addData(stamped_events, imu_stamps_vec, imu_accgyr_vec);
}

}  // namespace

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

#ifndef HAVE_METAVISION_SDK_IMPL
  std::cerr << "[Live] Metavision C++ SDK headers not available at compile time.\n";
  return 1;
#else
  ze::VisualOdometry vo;
  // Force event-only rig initialization for this tool.
  FLAGS_vio_use_events_and_images = false;
  FLAGS_vio_use_events = true;
  vo.initialize();
  vo.frontend()->stage_ = ze::FrontendStage::Initializing;

  TumWriter tum(FLAGS_elis_live_out_tum);
  vo.registerResultCallback([&](const int64_t timestamp_ns,
                                const Eigen::Quaterniond& orientation,
                                const Eigen::Vector3d& position,
                                const ze::FrontendStage stage,
                                const uint32_t num_tracked_features) {
    std::cout << "[Live] t_ns=" << timestamp_ns
              << " stage=" << static_cast<int>(stage)
              << " keypoints=" << num_tracked_features
              << " p=[" << position.transpose() << "]\n";
    tum.write(timestamp_ns, orientation, position);
  });

  std::atomic<bool> stop{false};
  std::atomic<int> frames_done{0};
  std::atomic<int> frames_enqueued{0};
  std::atomic<int> slices_dropped{0};
  std::atomic<bool> processing{false};

  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::deque<ze::EventArrayPtr> slice_queue;

  auto enqueue_slice = [&](ze::EventArrayPtr slice) {
    if (!slice || slice->empty())
    {
      return;
    }

    const int queue_limit = std::max(1, FLAGS_elis_live_queue_size);
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      while (static_cast<int>(slice_queue.size()) >= queue_limit)
      {
        slice_queue.pop_front();
        ++slices_dropped;
      }
      slice_queue.push_back(std::move(slice));
    }
    queue_cv.notify_one();
  };

  std::thread worker([&]() {
    while (!stop.load())
    {
      ze::EventArrayPtr slice;
      {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [&]() { return stop.load() || !slice_queue.empty(); });
        if (stop.load() && slice_queue.empty())
        {
          break;
        }
        slice = std::move(slice_queue.front());
        slice_queue.pop_front();
      }

      processing.store(true, std::memory_order_relaxed);
      process_event_slice(*vo.frontend(), std::move(slice));
      processing.store(false, std::memory_order_relaxed);

      const int n = ++frames_done;
      if (FLAGS_elis_live_max_frames > 0 && n >= FLAGS_elis_live_max_frames)
      {
        stop.store(true);
        queue_cv.notify_all();
        break;
      }
    }
  });

  Metavision::Camera camera;
  try
  {
    if (!FLAGS_elis_live_serial.empty())
    {
      std::cout << "[Live] Opening camera by serial: " << FLAGS_elis_live_serial << "\n";
      camera = Metavision::Camera::from_serial(FLAGS_elis_live_serial);
    }
    else
    {
      std::cout << "[Live] Opening first available camera...\n";
      camera = Metavision::Camera::from_first_available();
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "[Live] Failed to open camera: " << e.what() << "\n"
              << "[Live] Troubleshooting:\n"
              << "  - Ensure the container is started with USB passthrough: ./launch.sh --usb\n"
              << "  - Check camera visibility: metavision_platform_info / metavision_viewer\n";
    stop.store(true);
    queue_cv.notify_all();
    worker.join();
    return 1;
  }

  std::vector<dvs_msgs::Event> buffer;
  buffer.reserve(static_cast<size_t>(FLAGS_elis_live_events_per_frame));
  uint64_t slice_start_us = 0;
  bool slice_has_start = false;

  camera.cd().add_callback([&](const Metavision::EventCD* begin, const Metavision::EventCD* end) {
    if (stop.load())
    {
      return;
    }
    for (auto it = begin; it != end && !stop.load(); ++it)
    {
      if (!slice_has_start)
      {
        slice_start_us = static_cast<uint64_t>(it->t);
        slice_has_start = true;
      }

      dvs_msgs::Event ev;
      ev.x = static_cast<uint16_t>(it->x);
      ev.y = static_cast<uint16_t>(it->y);
      ev.ts.fromNSec(static_cast<uint64_t>(it->t) * 1000ull);
      ev.polarity = static_cast<bool>(it->p);
      buffer.push_back(ev);

      const bool time_slice_ready =
          (FLAGS_elis_live_slice_dt_us > 0) &&
          (static_cast<uint64_t>(it->t) >= slice_start_us + FLAGS_elis_live_slice_dt_us);
      const bool count_slice_ready =
          (FLAGS_elis_live_slice_dt_us == 0) &&
          (buffer.size() >= static_cast<size_t>(FLAGS_elis_live_events_per_frame));

      if (time_slice_ready || count_slice_ready)
      {
        auto slice = std::make_shared<ze::EventArray>();
        slice->swap(buffer);
        buffer.clear();
        buffer.reserve(static_cast<size_t>(FLAGS_elis_live_events_per_frame));
        slice_has_start = false;

        enqueue_slice(std::move(slice));
        const int n = ++frames_enqueued;
        if (FLAGS_elis_live_max_frames > 0 && n >= FLAGS_elis_live_max_frames)
        {
          stop.store(true);
          break;
        }
      }
    }
  });

  std::cout << "[Live] Camera started. Slicing by "
            << ((FLAGS_elis_live_slice_dt_us > 0)
                  ? (std::to_string(FLAGS_elis_live_slice_dt_us) + " us")
                  : (std::to_string(FLAGS_elis_live_events_per_frame) + " events"))
            << ". Press Ctrl+C to stop.\n";

  camera.start();
  while (camera.is_running() && !stop.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (camera.is_running())
  {
    camera.stop();
  }

  // Flush remaining events.
  if (!buffer.empty())
  {
    auto slice = std::make_shared<ze::EventArray>();
    slice->swap(buffer);
    enqueue_slice(std::move(slice));
  }

  stop.store(true);
  queue_cv.notify_all();
  worker.join();

  std::cout << "[Live] Done. Frames processed: " << frames_done.load()
            << " (enqueued=" << frames_enqueued.load()
            << ", dropped=" << slices_dropped.load() << ")\n";

  if (vo.frontend())
  {
    vo.frontend()->shutdown();
  }
  return 0;
#endif
}
