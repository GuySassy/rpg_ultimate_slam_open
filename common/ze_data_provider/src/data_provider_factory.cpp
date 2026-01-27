// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Modified: Robotics and Perception Group

#include <ze/common/logging.hpp>
#include <ze/data_provider/data_provider_factory.hpp>
#include <ze/data_provider/data_provider_base.hpp>
#include <ze/data_provider/data_provider_csv.hpp>
#include <ze/data_provider/data_provider_rosbag.hpp>
#include <ze/data_provider/data_provider_rostopic.hpp>
#include <ze/data_provider/data_provider_rt1060.hpp>

#include <algorithm>
#include <cctype>

DEFINE_string(bag_filename, "dataset.bag", "Name of bagfile in data_dir.");

DEFINE_string(topic_cam0, "/cam0/image_raw", "");
DEFINE_string(topic_cam1, "/cam1/image_raw", "");
DEFINE_string(topic_cam2, "/cam2/image_raw", "");
DEFINE_string(topic_cam3, "/cam2/image_raw", "");

DEFINE_string(topic_imu0, "/imu0", "");
DEFINE_string(topic_imu1, "/imu1", "");
DEFINE_string(topic_imu2, "/imu2", "");
DEFINE_string(topic_imu3, "/imu3", "");

DEFINE_string(topic_dvs0, "/dvs0/events", "");
DEFINE_string(topic_dvs1, "/dvs1/events", "");
DEFINE_string(topic_dvs2, "/dvs2/events", "");
DEFINE_string(topic_dvs3, "/dvs3/events", "");

DEFINE_string(topic_acc0, "/acc0", "");
DEFINE_string(topic_acc1, "/acc1", "");
DEFINE_string(topic_acc2, "/acc2", "");
DEFINE_string(topic_acc3, "/acc3", "");

DEFINE_string(topic_gyr0, "/gyr0", "");
DEFINE_string(topic_gyr1, "/gyr1", "");
DEFINE_string(topic_gyr2, "/gyr2", "");
DEFINE_string(topic_gyr3, "/gyr3", "");

DEFINE_int32(data_source, 1, " 0: CSV, 1: Rosbag, 2: Rostopic, 3: RT1060");
DEFINE_string(data_dir, "", "Directory for csv dataset.");
DEFINE_uint64(num_imus, 1, "Number of IMUs used in the pipeline.");
DEFINE_uint64(num_cams, 1, "Number of normal cameras used in the pipeline.");
DEFINE_uint64(num_dvs, 1, "Number of event cameras used in the pipeline.");
DEFINE_uint64(num_accels, 0, "Number of Accelerometers used in the pipeline.");
DEFINE_uint64(num_gyros, 0, "Number of Gyroscopes used in the pipeline.");
DEFINE_double(timeshift_cam_imu, 0., "Delay between cam and IMU timestamps.");

DEFINE_string(rt1060_port, "/dev/ttyACM0",
              "RT1060 serial device (e.g. /dev/ttyACM0).");
DEFINE_int32(rt1060_baud, 115200,
             "RT1060 serial baud rate (USB CDC ignores but required by API).");
DEFINE_string(rt1060_keypoint_source, "elis_code",
              "RT1060 keypoint source: elis_code or firmware.");
DEFINE_int32(rt1060_queue_size, 2,
             "RT1060 provider queue size for decoded slices.");
DEFINE_bool(rt1060_drop_empty_raw, true,
            "RT1060 provider drops packets with no raw events.");
DEFINE_string(rt1060_ts_anchor, "end",
              "RT1060 timestamp anchor for compact raw: start or end.");
DEFINE_string(rt1060_raw_format, "compact",
              "RT1060 raw format override: auto|compact|full.");
DEFINE_bool(rt1060_allow_xy_only_synth, false,
            "Allow RAW_XY_ONLY packets to synthesize events (test mode).");
DEFINE_int32(rt1060_xy_only_window_us, 3000,
             "Synthetic window (us) for RAW_XY_ONLY event timestamps.");
DEFINE_int32(rt1060_xy_only_polarity, 1,
             "Synthetic polarity for RAW_XY_ONLY events (0 or 1).");
DEFINE_int32(rt1060_log_stats_interval_s, 5,
             "Seconds between RT1060 parser stats logs (0 disables).");
DEFINE_bool(rt1060_debug_packets, false,
            "Log RT1060 packet debug details (throttled).");
DEFINE_int32(rt1060_debug_every_n_packets, 200,
             "Log one RT1060 packet every N parsed when debug is enabled.");
DEFINE_string(rt1060_debug_log_path, "",
              "Write RT1060 debug CSV to this path (empty disables).");
DEFINE_int32(rt1060_debug_log_max_lines, 2000,
             "Max RT1060 debug log lines (<=0 for unlimited).");
DEFINE_int32(rt1060_debug_log_flush_every, 20,
             "Flush RT1060 debug log every N lines.");

namespace {

ze::Rt1060TsAnchor parseRt1060TsAnchor(const std::string& value)
{
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lowered == "start")
  {
    return ze::Rt1060TsAnchor::Start;
  }
  if (lowered == "end")
  {
    return ze::Rt1060TsAnchor::End;
  }
  LOG(WARNING) << "Unknown --rt1060_ts_anchor=" << value
               << " (expected start|end); defaulting to end.";
  return ze::Rt1060TsAnchor::End;
}

ze::Rt1060RawFormat parseRt1060RawFormat(const std::string& value)
{
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lowered == "auto")
  {
    return ze::Rt1060RawFormat::Auto;
  }
  if (lowered == "full" || lowered == "1")
  {
    return ze::Rt1060RawFormat::Full;
  }
  if (lowered == "compact" || lowered == "0")
  {
    return ze::Rt1060RawFormat::Compact;
  }
  LOG(WARNING) << "Unknown --rt1060_raw_format=" << value
               << " (expected auto|compact|full); defaulting to compact.";
  return ze::Rt1060RawFormat::Compact;
}

}  // namespace

namespace ze {

DataProviderBase::Ptr loadDataProviderFromGflags(const uint32_t num_cams)
{
  CHECK_GT(FLAGS_num_cams, 0u);
  CHECK_LE(FLAGS_num_cams, 4u);
  CHECK_LE(FLAGS_num_imus, 4u);

  // Fill camera topics.
  std::map<std::string, size_t> cam_topics;
  if (FLAGS_num_cams >= 1) cam_topics[FLAGS_topic_cam0] = 0;
  if (FLAGS_num_cams >= 2) cam_topics[FLAGS_topic_cam1] = 1;
  if (FLAGS_num_cams >= 3) cam_topics[FLAGS_topic_cam2] = 2;
  if (FLAGS_num_cams >= 4) cam_topics[FLAGS_topic_cam3] = 3;

  // Fill imu topics.
  std::map<std::string, size_t> imu_topics;
  if (FLAGS_num_imus >= 1) imu_topics[FLAGS_topic_imu0] = 0;
  if (FLAGS_num_imus >= 2) imu_topics[FLAGS_topic_imu1] = 1;
  if (FLAGS_num_imus >= 3) imu_topics[FLAGS_topic_imu2] = 2;
  if (FLAGS_num_imus >= 4) imu_topics[FLAGS_topic_imu3] = 3;

  // Fill event camera topics.
  std::map<std::string, size_t> dvs_topics;
  if (FLAGS_num_dvs >= 1) dvs_topics[FLAGS_topic_dvs0] = 0;
  if (FLAGS_num_dvs >= 2) dvs_topics[FLAGS_topic_dvs1] = 1;
  if (FLAGS_num_dvs >= 3) dvs_topics[FLAGS_topic_dvs2] = 2;
  if (FLAGS_num_dvs >= 4) dvs_topics[FLAGS_topic_dvs3] = 3;

  // Fill accelerometer topics.
  std::map<std::string, size_t> acc_topics;
  if (FLAGS_num_accels >= 1) acc_topics[FLAGS_topic_acc0] = 0;
  if (FLAGS_num_accels >= 2) acc_topics[FLAGS_topic_acc1] = 1;
  if (FLAGS_num_accels >= 3) acc_topics[FLAGS_topic_acc2] = 2;
  if (FLAGS_num_accels >= 4) acc_topics[FLAGS_topic_acc3] = 3;

  // Fill gyroscope topics.
  std::map<std::string, size_t> gyr_topics;
  if (FLAGS_num_gyros >= 1) gyr_topics[FLAGS_topic_gyr0] = 0;
  if (FLAGS_num_gyros >= 2) gyr_topics[FLAGS_topic_gyr1] = 1;
  if (FLAGS_num_gyros >= 3) gyr_topics[FLAGS_topic_gyr2] = 2;
  if (FLAGS_num_gyros >= 4) gyr_topics[FLAGS_topic_gyr3] = 3;

  // Create data provider.
  ze::DataProviderBase::Ptr data_provider;
  LOG(INFO) << "FLAGS_data_source = " << FLAGS_data_source;
  switch (FLAGS_data_source)
  {
    case 0: // CSV
    {
      LOG(FATAL) << "data_provider_csv not supported";
      break;
    }
    case 1: // Rosbag
    {
      // Use the split imu dataprovider
      if (FLAGS_num_accels != 0 && FLAGS_num_gyros != 0)
      {
        LOG(FATAL) << "data_provider_rosbag with split IMU not supported";
      }
      else
      {
        data_provider.reset(new DataProviderRosbag(FLAGS_bag_filename,
                                                   imu_topics,
                                                   cam_topics,
                                                   dvs_topics));
      }
      break;
    }
    case 2: // Rostopic
    {
      if (FLAGS_num_accels != 0 && FLAGS_num_gyros != 0)
      {
        LOG(FATAL) << "Unsupported data provider";
      }
      else
      {
        data_provider.reset(new DataProviderRostopic(imu_topics,
                                                     cam_topics,
                                                     dvs_topics,
                                                     1000u,
                                                     1000u,
                                                     10000u,
                                                     100u));
      }

      break;
    }
    case 3: // RT1060
    {
      const bool use_firmware = (FLAGS_rt1060_keypoint_source == "firmware");
      data_provider.reset(new DataProviderRt1060(
        FLAGS_rt1060_port,
        FLAGS_rt1060_baud,
        use_firmware,
        FLAGS_rt1060_drop_empty_raw,
        FLAGS_rt1060_queue_size,
        FLAGS_rt1060_allow_xy_only_synth,
        FLAGS_rt1060_xy_only_window_us,
        FLAGS_rt1060_xy_only_polarity,
        parseRt1060TsAnchor(FLAGS_rt1060_ts_anchor),
        parseRt1060RawFormat(FLAGS_rt1060_raw_format),
        FLAGS_rt1060_log_stats_interval_s,
        FLAGS_rt1060_debug_packets,
        FLAGS_rt1060_debug_every_n_packets,
        FLAGS_rt1060_debug_log_path,
        FLAGS_rt1060_debug_log_max_lines,
        FLAGS_rt1060_debug_log_flush_every,
        static_cast<size_t>(FLAGS_num_imus)));
      break;
    }
    default:
    {
      LOG(FATAL) << "Data source not known.";
      break;
    }
  }

  return data_provider;
}

} // namespace ze
