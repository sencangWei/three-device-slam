// Binary stdin adapter for feeding an externally owned D435i stream to ORB-SLAM3.
// The process never opens librealsense; camera ownership remains in the Python
// acquisition process that also owns the D405.

#include <System.h>
#include <Tracking.h>

#include <opencv2/core.hpp>

#include <Eigen/Geometry>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <stdexcept>
#include <vector>

#include <unistd.h>

namespace {

constexpr std::array<char, 4> kInputMagic{{'O', 'R', 'B', 'I'}};
constexpr std::array<char, 4> kPoseMagic{{'O', 'R', 'B', 'P'}};
constexpr uint8_t kFrame = 1;
constexpr uint8_t kShutdown = 2;
constexpr uint32_t kExpectedWidth = 1280;
constexpr uint32_t kExpectedHeight = 720;
constexpr uint32_t kMaximumImuSamples = 4096;

#pragma pack(push, 1)
struct InputHeader {
    char magic[4];
    uint8_t type;
    uint8_t reserved[3];
    int64_t timestamp_ns;
    uint32_t width;
    uint32_t height;
    uint32_t imu_count;
    uint32_t left_bytes;
    uint32_t right_bytes;
};

struct ImuWire {
    int64_t timestamp_ns;
    double accel[3];
    double gyro[3];
};

struct PoseWire {
    char magic[4];
    int64_t timestamp_ns;
    int32_t tracking_state;
    uint32_t imu_count;
    double values[9];  // tx,ty,tz,qx,qy,qz,qw,TrackStereo milliseconds, inertial BA2 ready
};
#pragma pack(pop)

static_assert(sizeof(InputHeader) == 36, "unexpected InputHeader packing");
static_assert(sizeof(ImuWire) == 56, "unexpected ImuWire packing");
static_assert(sizeof(PoseWire) == 92, "unexpected PoseWire packing");

bool read_exact(int fd, void* destination, size_t size) {
    auto* cursor = static_cast<unsigned char*>(destination);
    size_t completed = 0;
    while (completed < size) {
        const ssize_t count = ::read(fd, cursor + completed, size - completed);
        if (count == 0) {
            return false;
        }
        if (count < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error("read from input pipe failed");
        }
        completed += static_cast<size_t>(count);
    }
    return true;
}

void write_exact(int fd, const void* source, size_t size) {
    const auto* cursor = static_cast<const unsigned char*>(source);
    size_t completed = 0;
    while (completed < size) {
        const ssize_t count = ::write(fd, cursor + completed, size - completed);
        if (count < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error("write to pose pipe failed");
        }
        completed += static_cast<size_t>(count);
    }
}

PoseWire make_pose(int64_t timestamp_ns, int state, uint32_t imu_count,
                   const Sophus::SE3f& camera_from_world, double processing_ms,
                   bool inertial_ba2_ready) {
    PoseWire result{};
    std::memcpy(result.magic, kPoseMagic.data(), kPoseMagic.size());
    result.timestamp_ns = timestamp_ns;
    result.tracking_state = state;
    result.imu_count = imu_count;
    result.values[7] = processing_ms;
    result.values[8] = inertial_ba2_ready ? 1.0 : 0.0;
    if (state == ORB_SLAM3::Tracking::OK || state == ORB_SLAM3::Tracking::OK_KLT) {
        const Sophus::SE3f world_from_camera = camera_from_world.inverse();
        const Eigen::Vector3f translation = world_from_camera.translation();
        const Eigen::Quaternionf quaternion = world_from_camera.unit_quaternion();
        result.values[0] = translation.x();
        result.values[1] = translation.y();
        result.values[2] = translation.z();
        result.values[3] = quaternion.x();
        result.values[4] = quaternion.y();
        result.values[5] = quaternion.z();
        result.values[6] = quaternion.w();
    } else {
        result.values[6] = 1.0;
    }
    return result;
}

void send_ready(int pose_fd) {
    PoseWire ready{};
    std::memcpy(ready.magic, kPoseMagic.data(), kPoseMagic.size());
    ready.timestamp_ns = -1;
    ready.tracking_state = ORB_SLAM3::Tracking::NO_IMAGES_YET;
    ready.values[6] = 1.0;
    write_exact(pose_fd, &ready, sizeof(ready));
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "usage: orbslam3_live_adapter VOCAB SETTINGS OUTPUT_DIR POSE_FD\n";
        return 2;
    }
    const std::string vocabulary = argv[1];
    const std::string settings = argv[2];
    const std::filesystem::path output = std::filesystem::absolute(argv[3]);
    const int pose_fd = std::stoi(argv[4]);
    std::filesystem::create_directories(output);
    std::filesystem::current_path(output);

    try {
        ORB_SLAM3::System slam(
            vocabulary, settings, ORB_SLAM3::System::IMU_STEREO, false);
        send_ready(pose_fd);

        int64_t last_frame_ns = std::numeric_limits<int64_t>::min();
        bool had_tracking_ok = false;
        while (true) {
            InputHeader header{};
            if (!read_exact(STDIN_FILENO, &header, sizeof(header))) {
                break;
            }
            if (std::memcmp(header.magic, kInputMagic.data(), kInputMagic.size()) != 0) {
                throw std::runtime_error("invalid input packet magic");
            }
            if (header.type == kShutdown) {
                break;
            }
            const uint32_t expected_bytes = kExpectedWidth * kExpectedHeight;
            if (header.type != kFrame || header.width != kExpectedWidth ||
                header.height != kExpectedHeight || header.left_bytes != expected_bytes ||
                header.right_bytes != expected_bytes || header.imu_count > kMaximumImuSamples ||
                header.timestamp_ns <= last_frame_ns) {
                throw std::runtime_error("invalid or non-monotonic frame packet");
            }
            std::vector<ImuWire> wire_imu(header.imu_count);
            if (!wire_imu.empty() &&
                !read_exact(STDIN_FILENO, wire_imu.data(), wire_imu.size() * sizeof(ImuWire))) {
                throw std::runtime_error("truncated IMU payload");
            }
            std::vector<uint8_t> left(header.left_bytes);
            std::vector<uint8_t> right(header.right_bytes);
            if (!read_exact(STDIN_FILENO, left.data(), left.size()) ||
                !read_exact(STDIN_FILENO, right.data(), right.size())) {
                throw std::runtime_error("truncated stereo payload");
            }

            std::vector<ORB_SLAM3::IMU::Point> measurements;
            measurements.reserve(wire_imu.size());
            int64_t previous_imu_ns = std::numeric_limits<int64_t>::min();
            for (const ImuWire& sample : wire_imu) {
                if (sample.timestamp_ns <= previous_imu_ns ||
                    sample.timestamp_ns > header.timestamp_ns) {
                    throw std::runtime_error("invalid IMU timestamp order");
                }
                previous_imu_ns = sample.timestamp_ns;
                const cv::Point3f accel(
                    static_cast<float>(sample.accel[0]),
                    static_cast<float>(sample.accel[1]),
                    static_cast<float>(sample.accel[2]));
                const cv::Point3f gyro(
                    static_cast<float>(sample.gyro[0]),
                    static_cast<float>(sample.gyro[1]),
                    static_cast<float>(sample.gyro[2]));
                const double timestamp = static_cast<double>(sample.timestamp_ns) * 1e-9;
                measurements.emplace_back(accel, gyro, timestamp);
            }

            cv::Mat left_image(
                static_cast<int>(header.height), static_cast<int>(header.width), CV_8UC1,
                left.data());
            cv::Mat right_image(
                static_cast<int>(header.height), static_cast<int>(header.width), CV_8UC1,
                right.data());
            const double frame_time = static_cast<double>(header.timestamp_ns) * 1e-9;
            const auto started = std::chrono::steady_clock::now();
            const Sophus::SE3f camera_from_world =
                slam.TrackStereo(left_image, right_image, frame_time, measurements);
            const double processing_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
            const int state = slam.GetTrackingState();
            had_tracking_ok = had_tracking_ok || state == ORB_SLAM3::Tracking::OK ||
                              state == ORB_SLAM3::Tracking::OK_KLT;
            const PoseWire pose = make_pose(
                header.timestamp_ns, state, header.imu_count, camera_from_world,
                processing_ms, slam.IsInertialBA2Complete());
            write_exact(pose_fd, &pose, sizeof(pose));
            last_frame_ns = header.timestamp_ns;
        }

        slam.Shutdown();
        if (had_tracking_ok) {
            slam.SaveTrajectoryEuRoC((output / "f_d435i_ego.txt").string());
            slam.SaveKeyFrameTrajectoryEuRoC((output / "kf_d435i_ego.txt").string());
        }
        ::close(pose_fd);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "orbslam3_live_adapter: " << error.what() << '\n';
        ::close(pose_fd);
        return 1;
    }
}
