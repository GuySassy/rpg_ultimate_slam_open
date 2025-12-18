// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <gflags/gflags.h>
#include <ze/common/logging.hpp>

#include <ze/common/types.hpp>
#include <ze/vio_frontend/frontend_api.hpp>
#include <ze/vio_frontend/frontend_base.hpp>

#include <dvs_msgs/Event.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#if defined(HAVE_METAVISION_SDK) && __has_include(<metavision/sdk/base/events/event_cd.h>) && __has_include(<metavision/sdk/driver/camera.h>)
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/driver/camera.h>
#define HAVE_METAVISION_SDK_IMPL 1
#else
#undef HAVE_METAVISION_SDK_IMPL
#endif

DEFINE_string(elis_playback_event_file, "",
              "Path to a Metavision .raw recording to replay through the frontend.");
DEFINE_uint64(elis_playback_events_per_frame, 50000,
              "Number of events per frontend frame.");
DEFINE_uint64(elis_playback_slice_dt_us, 10000,
              "If >0, slice the RAW stream into fixed time windows (microseconds) "
              "instead of fixed event counts. Set to 0 to use --elis_playback_events_per_frame.");
DEFINE_int32(elis_playback_max_frames, 0,
             "Maximum number of frames to process (0 = until end-of-file).");
DEFINE_int32(elis_playback_idle_timeout_ms, 2000,
             "Stop playback if no new events arrive for this many milliseconds. "
             "Useful to detect end-of-file in some SDK versions.");
DEFINE_string(elis_playback_out_tum, "",
              "Optional output path for a TUM trajectory log (t[s] tx ty tz qx qy qz qw).");

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
        std::cerr << "[Playback] Failed to open TUM output: " << path << "\n";
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
  // In offline playback we don't have real IMU, so we stitch synthetic stamps
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

  if (FLAGS_elis_playback_event_file.empty())
  {
    std::cerr << "Usage: rosrun ze_vio_frontend elis_link_playback "
                 "--elis_playback_event_file=/path/to/file.raw "
                 "[--elis_playback_slice_dt_us=10000] "
                 "[--elis_playback_events_per_frame=50000 (if slice_dt_us=0)] "
                 "[--elis_playback_max_frames=0] "
                 "[--elis_playback_idle_timeout_ms=2000] "
                 "[--elis_playback_out_tum=/tmp/traj.txt] "
                 "[--calib_filename=... --vio_use_events=true --vio_use_elis_link=true]\n";
    return 2;
  }

#ifndef HAVE_METAVISION_SDK_IMPL
  std::cerr << "[Playback] Metavision C++ SDK headers not available at compile time.\n";
  return 1;
#else
  ze::VisualOdometry vo;
  // Force event-only rig initialization for this playback tool.
  FLAGS_vio_use_events_and_images = false;
  FLAGS_vio_use_events = true;
  vo.initialize();
  vo.frontend()->stage_ = ze::FrontendStage::Initializing;

  TumWriter tum(FLAGS_elis_playback_out_tum);
  vo.registerResultCallback([&](const int64_t timestamp_ns,
                                const Eigen::Quaterniond& orientation,
                                const Eigen::Vector3d& position,
                                const ze::FrontendStage stage,
                                const uint32_t num_tracked_features) {
    std::cout << "[Playback] t_ns=" << timestamp_ns
              << " stage=" << static_cast<int>(stage)
              << " keypoints=" << num_tracked_features
              << " p=[" << position.transpose() << "]\n";
    tum.write(timestamp_ns, orientation, position);
  });

  std::atomic<int> frames_done{0};
  std::atomic<bool> stop{false};
  std::atomic<bool> processing{false};
  std::atomic<bool> reached_max_frames{false};

  const auto now_us = []() -> int64_t {
    return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
  };
  std::atomic<int64_t> last_activity_us{now_us()};
  std::vector<dvs_msgs::Event> buffer;
  buffer.reserve(static_cast<size_t>(FLAGS_elis_playback_events_per_frame));
  uint64_t slice_start_us = 0;
  bool slice_has_start = false;

  Metavision::Camera camera = Metavision::Camera::from_file(FLAGS_elis_playback_event_file);
  camera.cd().add_callback([&](const Metavision::EventCD* begin, const Metavision::EventCD* end) {
    last_activity_us.store(now_us(), std::memory_order_relaxed);
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
          (FLAGS_elis_playback_slice_dt_us > 0) &&
          (static_cast<uint64_t>(it->t) >= slice_start_us + FLAGS_elis_playback_slice_dt_us);
      const bool count_slice_ready =
          (FLAGS_elis_playback_slice_dt_us == 0) &&
          (buffer.size() >= static_cast<size_t>(FLAGS_elis_playback_events_per_frame));

      if (time_slice_ready || count_slice_ready)
      {
        auto slice = std::make_shared<ze::EventArray>();
        slice->swap(buffer);
        buffer.clear();
        buffer.reserve(static_cast<size_t>(FLAGS_elis_playback_events_per_frame));
        slice_has_start = false;

        processing.store(true, std::memory_order_relaxed);
        last_activity_us.store(now_us(), std::memory_order_relaxed);
        process_event_slice(*vo.frontend(), slice);
        last_activity_us.store(now_us(), std::memory_order_relaxed);
        processing.store(false, std::memory_order_relaxed);

        const int n = ++frames_done;
        if (FLAGS_elis_playback_max_frames > 0 && n >= FLAGS_elis_playback_max_frames)
        {
          reached_max_frames.store(true, std::memory_order_relaxed);
          stop.store(true);
          break;
        }
      }
    }
  });

  std::cout << "[Playback] Starting Metavision playback: " << FLAGS_elis_playback_event_file << "\n";
  if (FLAGS_elis_playback_slice_dt_us > 0)
  {
    std::cout << "[Playback] Slicing by time: " << FLAGS_elis_playback_slice_dt_us << " us\n";
  }
  else
  {
    std::cout << "[Playback] Slicing by count: " << FLAGS_elis_playback_events_per_frame << " events\n";
  }
  camera.start();

  while (camera.is_running() && !stop.load())
  {
    if (FLAGS_elis_playback_idle_timeout_ms > 0 && !processing.load(std::memory_order_relaxed))
    {
      const int64_t idle_us =
          now_us() - last_activity_us.load(std::memory_order_relaxed);
      if (idle_us > static_cast<int64_t>(FLAGS_elis_playback_idle_timeout_ms) * 1000)
      {
        std::cout << "[Playback] No new events for "
                  << (idle_us / 1000) << " ms; stopping (EOF or stalled).\n";
        stop.store(true);
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (camera.is_running())
  {
    camera.stop();
  }

  // Flush remaining events as a final slice (EOF / partial frame).
  const bool allow_partial_flush =
      !reached_max_frames.load(std::memory_order_relaxed) &&
      (FLAGS_elis_playback_max_frames <= 0 ||
       frames_done.load(std::memory_order_relaxed) < FLAGS_elis_playback_max_frames);
  if (allow_partial_flush && !buffer.empty())
  {
    auto slice = std::make_shared<ze::EventArray>();
    slice->swap(buffer);
    process_event_slice(*vo.frontend(), slice);
    ++frames_done;
  }

  std::cout << "[Playback] Done. Frames processed: " << frames_done.load() << "\n";
  // VisualOdometry::shutdown() requires a DataProvider (subscribeDataProviders()).
  // This playback tool drives the frontend directly, so shut it down explicitly.
  if (vo.frontend())
  {
    vo.frontend()->shutdown();
  }
  return 0;
#endif
}
