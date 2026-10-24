// Reads "i <receive_ns> <stamp_ns>" (IMU: feeds the estimate) or "c <receive_ns> <stamp_ns>" (cloud: converted only)
// lines on stdin, in receive order, and prints the converted host stamp per line ("none" before the first IMU).
// Same calls as StateBridge::on_imu / on_cloud; driven by scripts/test_clock_offset.py.
#include <cstdint>
#include <iostream>
#include <string>

#include "g1_bringup/clock_offset.hpp"

int main(int argc, char ** argv)
{
  g1_bringup::ClockOffset clock(argc > 1 ? std::stod(argv[1]) : 2.0);
  std::ios::sync_with_stdio(false);
  char kind;
  int64_t receive_ns, stamp_ns;
  while (std::cin >> kind >> receive_ns >> stamp_ns) {
    if (kind == 'i') {
      clock.add(receive_ns, stamp_ns);
    }
    auto host = clock.to_host_ns(stamp_ns);
    if (host) {
      std::cout << *host << '\n';
    } else {
      std::cout << "none\n";
    }
  }
  return 0;
}
