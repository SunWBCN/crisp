// Generate a translation-only nominal TCP trajectory with standard quintic time scaling.
//
// The x/y/z inputs and CSV positions are ABSOLUTE TCP coordinates in the robot world/base
// frame, in metres. They are not per-cycle increments. This utility does not generate an
// orientation trajectory: the caller/controller should keep one fixed desired orientation
// for every row while consuming the generated positions.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

constexpr double kDefaultSampleHz = 1000.0;
constexpr std::uint64_t kMaxIntervals = 10000000;

struct Vec3 {
  double x;
  double y;
  double z;
};

Vec3 operator+(const Vec3& lhs, const Vec3& rhs) {
  return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 operator-(const Vec3& lhs, const Vec3& rhs) {
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 operator*(const Vec3& value, double scale) {
  return {value.x * scale, value.y * scale, value.z * scale};
}

void printUsage(const char* program) {
  std::cerr
      << "Usage:\n"
      << "  " << program
      << " duration x0 y0 z0 x1 y1 z1 [sample_hz] [output.csv]\n\n"
      << "Arguments:\n"
      << "  duration   Motion duration in seconds; must be > 0.\n"
      << "  x0 y0 z0  Initial absolute TCP position in world/base frame [m].\n"
      << "  x1 y1 z1  Final absolute TCP position in world/base frame [m].\n"
      << "  sample_hz  Requested sample rate [Hz], default 1000. The interval count\n"
      << "             is rounded up so both endpoints are included exactly.\n"
      << "  output.csv Output path. If omitted or '-', CSV is written to stdout.\n\n"
      << "The CSV contains a translation-only nominal trajectory. Keep the desired TCP\n"
      << "orientation fixed separately. Each x/y/z row is an absolute world-frame target,\n"
      << "not a displacement to add to the previous row.\n";
}

double parseFiniteDouble(const char* text, const char* name) {
  try {
    std::size_t parsed = 0;
    const std::string input(text);
    const double value = std::stod(input, &parsed);
    if (parsed != input.size() || !std::isfinite(value)) {
      throw std::invalid_argument("not a finite number");
    }
    return value;
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string("Invalid ") + name + ": '" + text + "'.");
  }
}

void writeSample(std::ostream& out,
                 double time,
                 double duration,
                 const Vec3& start,
                 const Vec3& displacement) {
  const double u = std::clamp(time / duration, 0.0, 1.0);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;

  // s(u) = 10u^3 - 15u^4 + 6u^5, with zero velocity and acceleration at u=0,1.
  const double s = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
  const double ds_du = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
  const double d2s_du2 = 60.0 * u - 180.0 * u2 + 120.0 * u3;
  const double s_dot = ds_du / duration;
  const double s_ddot = d2s_du2 / (duration * duration);

  const Vec3 position = start + displacement * s;
  const Vec3 velocity = displacement * s_dot;
  const Vec3 acceleration = displacement * s_ddot;

  out << time << ',' << u << ',' << s << ',' << s_dot << ',' << s_ddot << ','
      << position.x << ',' << position.y << ',' << position.z << ','
      << velocity.x << ',' << velocity.y << ',' << velocity.z << ','
      << acceleration.x << ',' << acceleration.y << ',' << acceleration.z << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc == 2 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
    printUsage(argv[0]);
    return 0;
  }
  if (argc < 8 || argc > 10) {
    printUsage(argv[0]);
    return 2;
  }

  try {
    const double duration = parseFiniteDouble(argv[1], "duration");
    const Vec3 start{
        parseFiniteDouble(argv[2], "x0"),
        parseFiniteDouble(argv[3], "y0"),
        parseFiniteDouble(argv[4], "z0")};
    const Vec3 finish{
        parseFiniteDouble(argv[5], "x1"),
        parseFiniteDouble(argv[6], "y1"),
        parseFiniteDouble(argv[7], "z1")};
    const double sample_hz =
        argc >= 9 ? parseFiniteDouble(argv[8], "sample_hz") : kDefaultSampleHz;

    if (duration <= 0.0) {
      throw std::invalid_argument("duration must be greater than zero.");
    }
    if (sample_hz <= 0.0) {
      throw std::invalid_argument("sample_hz must be greater than zero.");
    }

    const long double requested_intervals =
        static_cast<long double>(duration) * static_cast<long double>(sample_hz);
    if (!std::isfinite(requested_intervals) ||
        requested_intervals > static_cast<long double>(kMaxIntervals)) {
      throw std::invalid_argument(
          "duration * sample_hz is too large; at most 10,000,000 intervals are allowed.");
    }

    const std::uint64_t interval_count = std::max<std::uint64_t>(
        1, static_cast<std::uint64_t>(std::ceil(requested_intervals)));
    const double sample_period = duration / static_cast<double>(interval_count);

    std::ofstream output_file;
    std::ostream* output = &std::cout;
    const std::string output_path = argc == 10 ? argv[9] : "-";
    if (output_path != "-") {
      output_file.open(output_path, std::ios::out | std::ios::trunc);
      if (!output_file) {
        throw std::runtime_error("Could not open output file: '" + output_path + "'.");
      }
      output = &output_file;
    }

    *output << std::fixed << std::setprecision(10);
    *output << "time_s,u,s,s_dot_per_s,s_ddot_per_s2,"
               "tcp_x_m,tcp_y_m,tcp_z_m,"
               "tcp_vx_mps,tcp_vy_mps,tcp_vz_mps,"
               "tcp_ax_mps2,tcp_ay_mps2,tcp_az_mps2\n";

    const Vec3 displacement = finish - start;
    for (std::uint64_t i = 0; i <= interval_count; ++i) {
      const double time =
          i == interval_count ? duration : static_cast<double>(i) * sample_period;
      writeSample(*output, time, duration, start, displacement);
    }

    output->flush();
    if (!*output) {
      throw std::runtime_error("Failed while writing trajectory CSV.");
    }

    std::cerr << "Generated " << (interval_count + 1) << " samples over " << duration
              << " s (effective rate " << (static_cast<double>(interval_count) / duration)
              << " Hz)";
    if (output_path != "-") {
      std::cerr << " in '" << output_path << "'";
    }
    std::cerr << ".\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 2;
  }
}
