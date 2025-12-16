#include <elisUltimateLink/frame_populator.hpp>
#include <elisUltimateLink/keypoint_service.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>

#if defined(HAVE_METAVISION_SDK) && __has_include(<metavision/sdk/base/events/event_cd.h>) && __has_include(<metavision/sdk/driver/camera.h>)
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/driver/camera.h>
#define HAVE_METAVISION_SDK_IMPL 1
#else
#undef HAVE_METAVISION_SDK_IMPL
#endif

namespace {

struct Options
{
  std::string event_file;
  std::size_t max_events = 5000;
};

Options parse_options(int argc, char** argv)
{
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg.rfind("--event_file=", 0) == 0) {
      opt.event_file = arg.substr(std::string("--event_file=").size());
    } else if (arg.rfind("--max_events=", 0) == 0) {
      opt.max_events = static_cast<std::size_t>(std::stoul(arg.substr(std::string("--max_events=").size())));
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: elis_link_harness [--event_file=path] [--max_events=N]\n"
                   "Reads a small event batch (CSV: x,y,t,p) or generates synthetic events,\n"
                   "passes them through ElisKeypointService, and prints a summary of the\n"
                   "FrameFeatureData handed to the frontend.\n";
      std::exit(0);
    }
  }
  return opt;
}

bool load_events_raw_cpp(const std::string& path,
                         std::size_t max_events,
                         std::vector<int32_t>& xs,
                         std::vector<int32_t>& ys,
                         std::vector<float>& ts,
                         std::vector<int32_t>& ps)
{
#ifdef HAVE_METAVISION_SDK_IMPL
  try {
    Metavision::Camera camera = Metavision::Camera::from_file(path);
    std::atomic<bool> done{false};

    camera.cd().add_callback([&](const Metavision::EventCD* begin, const Metavision::EventCD* end) {
      for (auto it = begin; it != end && xs.size() < max_events; ++it) {
        xs.push_back(static_cast<int32_t>(it->x));
        ys.push_back(static_cast<int32_t>(it->y));
        ts.push_back(static_cast<float>(it->t));
        ps.push_back(static_cast<int32_t>(it->p));
      }
      if (xs.size() >= max_events) {
        done.store(true);
      }
    });

    camera.start();
    while (camera.is_running() && !done.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (camera.is_running()) {
      camera.stop();
    }

    std::cout << "[Harness] Loaded " << xs.size() << " events from RAW (C++): " << path << "\n";
    return !xs.empty();
  } catch (const std::exception& e) {
    std::cerr << "[Harness] Failed to load RAW via Metavision C++ API: " << e.what() << "\n";
    return false;
  }
#else
  (void)path;
  (void)max_events;
  (void)xs;
  (void)ys;
  (void)ts;
  (void)ps;
  std::cerr << "[Harness] Metavision SDK headers not available; cannot load RAW via C++.\n";
  return false;
#endif
}

bool load_events_csv(const std::string& path,
                     std::vector<int32_t>& xs,
                     std::vector<int32_t>& ys,
                     std::vector<float>& ts,
                     std::vector<int32_t>& ps,
                     std::size_t max_events)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    std::cerr << "[Harness] Failed to open event file: " << path << "\n";
    return false;
  }

  std::string line;
  while (xs.size() < max_events && std::getline(f, line)) {
    if (line.empty()) {
      continue;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream iss(line);
    int32_t x, y, p;
    float t;
    if (!(iss >> x >> y >> t >> p)) {
      continue;
    }
    xs.push_back(x);
    ys.push_back(y);
    ts.push_back(t);
    ps.push_back(p);
  }
  std::cout << "[Harness] Loaded " << xs.size() << " events from " << path << "\n";
  return !xs.empty();
}

void generate_synthetic(std::vector<int32_t>& xs,
                        std::vector<int32_t>& ys,
                        std::vector<float>& ts,
                        std::vector<int32_t>& ps,
                        std::size_t max_events)
{
  xs.reserve(max_events);
  ys.reserve(max_events);
  ts.reserve(max_events);
  ps.reserve(max_events);
  for (std::size_t i = 0; i < max_events; ++i) {
    xs.push_back(static_cast<int32_t>(i % 320));
    ys.push_back(static_cast<int32_t>((i / 5) % 240));
    ts.push_back(static_cast<float>(i) * 1e3f);  // microseconds
    ps.push_back((i % 2) ? 1 : 0);
  }
  std::cout << "[Harness] Generated " << xs.size() << " synthetic events.\n";
}

void summarize(const elis_link::KeypointPacket& pkt,
               const elis_link::FrameFeatureData& frame)
{
  std::cout << "[Harness] Keypoints: " << pkt.size()
            << "  stride: " << pkt.descriptor_stride
            << "  descriptors bytes: " << pkt.descriptors.size()
            << "  processing_time_ms: " << std::fixed << std::setprecision(3)
            << pkt.processing_time_ms << "\n";

  std::cout << "[Harness] FrameFeatureData -> keypoints cols: "
            << frame.keypoints_px.cols() << "  descriptors cols: "
            << frame.descriptors.cols() << " stride: "
            << frame.descriptor_stride << "\n";
}

}  // namespace

int main(int argc, char** argv)
{
#ifndef ELIS_LINK_ENABLED
  std::cerr << "[Harness] ELIS_LINK_ENABLED not defined; rebuild with elis_link_core available.\n";
  return 1;
#else
  const Options opt = parse_options(argc, argv);

  elis_link::ElisKeypointService service;
  elis_link::KeypointServiceConfig cfg;
  service.configure(cfg);

  std::vector<int32_t> xs, ys, ps;
  std::vector<float> ts;
  bool loaded = false;
  if (!opt.event_file.empty()) {
    const bool is_raw = opt.event_file.size() > 4 &&
                        (opt.event_file.find(".raw") != std::string::npos ||
                         opt.event_file.find(".dat") != std::string::npos ||
                         opt.event_file.find(".h5") != std::string::npos ||
                         opt.event_file.find(".hdf5") != std::string::npos);
    if (is_raw) {
      loaded = load_events_raw_cpp(opt.event_file, opt.max_events, xs, ys, ts, ps);
    } else {
      loaded = load_events_csv(opt.event_file, xs, ys, ts, ps, opt.max_events);
    }
  }
  if (!loaded) {
    generate_synthetic(xs, ys, ts, ps, opt.max_events);
  }

  elis_link::KeypointPacket pkt = service.process_batch(xs, ys, ts, ps);

  elis_link::FramePopulationConfig frame_cfg;
  frame_cfg.descriptor_length = pkt.descriptor_stride > 0 ? pkt.descriptor_stride : frame_cfg.descriptor_length;
  elis_link::FrameFeatureData frame = elis_link::make_frame_feature_data(pkt, frame_cfg);

  summarize(pkt, frame);
  return 0;
#endif
}
