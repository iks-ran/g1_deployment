// Reads lines on stdin, in receive order:
//   "i <stamp_ns> <wx> <wy> <wz>"                         lidar IMU sample: feeds the gyro buffer
//   "c <stamp_ns> <in.bin> <out.bin> [<step_s> <n> <x y z> * n]"  cloud (+ optional leg-odometry translation)
// in.bin = packed MID-360 points (22 bytes: x y z intensity ring time). Each cloud goes through the same
// deskew_to_end() call as StateBridge::try_deskew; out.bin = x y z intensity (16 bytes per point, scan-end frame).
// Prints "ok" or the reason nothing was written. Driven by scripts/test_deskew.py and scripts/sim_deskew_bench.py.
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

#include "g1_bringup/deskew.hpp"

int main(int argc, char ** argv)
{
  using sensor_msgs::msg::PointField;
  g1_bringup::GyroBuffer gyro(1000000000LL);
  const double time_scale = argc > 1 ? std::stod(argv[1]) : 1e-9;
  sensor_msgs::msg::PointCloud2 cloud;
  auto add = [&](const char * name, uint32_t off, uint8_t type) {
      PointField f;
      f.name = name;
      f.offset = off;
      f.datatype = type;
      f.count = 1;
      cloud.fields.push_back(f);
    };
  add("x", 0, PointField::FLOAT32);
  add("y", 4, PointField::FLOAT32);
  add("z", 8, PointField::FLOAT32);
  add("intensity", 12, PointField::FLOAT32);
  add("ring", 16, PointField::UINT16);
  add("time", 18, PointField::FLOAT32);
  cloud.point_step = 22;
  cloud.height = 1;
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream ss(line);
    char kind;
    int64_t stamp;
    if (!(ss >> kind >> stamp)) {
      continue;
    }
    if (kind == 'i') {
      double x, y, z;
      ss >> x >> y >> z;
      gyro.add(stamp, {x, y, z});
      continue;
    }
    std::string in, out;
    ss >> in >> out;
    g1_bringup::Translation trans;
    size_t n = 0;
    const bool have_trans = static_cast<bool>(ss >> trans.step_s >> n);
    trans.t.resize(n);
    for (auto & v : trans.t) {
      ss >> v[0] >> v[1] >> v[2];
    }
    std::ifstream f(in, std::ios::binary);
    cloud.data.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    cloud.width = static_cast<uint32_t>(cloud.data.size() / cloud.point_step);
    cloud.row_step = cloud.width * cloud.point_step;
    sensor_msgs::msg::PointCloud2 result;
    const std::string err = g1_bringup::deskew_to_end(cloud, result, stamp, gyro, time_scale,
        have_trans ? &trans : nullptr);
    std::ofstream(out, std::ios::binary).write(reinterpret_cast<const char *>(result.data.data()),
      static_cast<std::streamsize>(result.data.size()));
    std::cout << (err.empty() ? "ok" : err) << '\n';
  }
  return 0;
}
