#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "DualBoard.hpp"
#include "HostDataLink.hpp"

using Pldx::HostDataLink::ChassisTarget;
using namespace Pldx::HostDataLinkDetail;
using namespace Pldx::DualBoardControl;

int main() {
  ChassisTarget input{};
  input.vx_mps = 1.5F;
  input.vy_mps = -2.5F;
  input.wz_rad_s = 0.25F;
  input.use_speed_control = true;

  std::array<uint8_t, sizeof(input)> bytes{};
  std::memcpy(bytes.data(), &input, bytes.size());
  assert(bytes.size() == 13U && bytes[12] == 1U);
  assert(ChassisPayloadValid(LibXR::ConstRawData(bytes.data(), bytes.size())));
  assert(!ChassisPayloadValid(LibXR::ConstRawData(bytes.data(), 12U)));
  bytes[12] = 2U;
  assert(!ChassisPayloadValid(LibXR::ConstRawData(bytes.data(), bytes.size())));
  assert(!ChassisPayloadValid(LibXR::ConstRawData(nullptr, bytes.size())));

  ChassisCommandTracker tracker;
  assert(tracker.Observe(input, LibXR::MillisecondTimestamp(100U)));
  assert(tracker.CommandFresh(LibXR::MillisecondTimestamp(249U)));
  const CMD::Data accepted = MakeAIChassis(
      input, tracker.CommandFresh(LibXR::MillisecondTimestamp(100U)));
  assert(accepted.chassis_online && !accepted.chassis.ff_valid);
  for (Mode mode : {Mode::NAVIGATION, Mode::NAVIGATION_ROTOR}) {
    Command command{};
    command.vx = accepted.chassis.vx;
    command.vy = accepted.chassis.vy;
    command.wz = accepted.chassis.wz;
    command.mode = mode;
    Frame frame{};
    assert(Encode(command, frame));
    Command decoded{};
    assert(Decode(frame, decoded));
    assert(decoded.mode == mode);
    assert(std::fabs(decoded.vx - 1.5F) < 0.001F);
    assert(std::fabs(decoded.vy + 2.5F) < 0.001F);
    assert(std::fabs(decoded.wz - 0.25F) < 0.001F);
  }
  static_assert(AUTO_CHASSIS_MODE_ROTOR ==
                static_cast<uint32_t>(Mode::NAVIGATION_ROTOR));
  assert(!tracker.CommandFresh(LibXR::MillisecondTimestamp(250U)));
  const CMD::Data expired = MakeAIChassis(
      input, tracker.CommandFresh(LibXR::MillisecondTimestamp(250U)));
  assert(!expired.chassis_online && expired.chassis.vx == 0.0F &&
         expired.chassis.wz == 0.0F);

  input.vx_mps = std::numeric_limits<float>::quiet_NaN();
  assert(!tracker.Observe(input, LibXR::MillisecondTimestamp(251U)));
  assert(!tracker.CommandFresh(LibXR::MillisecondTimestamp(251U)));
  input.vx_mps = 1.5F;
  input.use_speed_control = false;
  assert(!ChassisCommandValid(input));
  input.use_speed_control = true;
  input.vx_mps = 33.0F;
  assert(!ChassisCommandValid(input));
  return 0;
}
