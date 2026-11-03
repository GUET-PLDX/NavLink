#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: ROS2_LIBXR navigation link protocol and host endpoint
depends:
- id: GUET-PLDX/CMD
- id: GUET-PLDX/Referee
=== END MANIFEST === */
// clang-format on

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"

namespace Pldx::HostDataLink {

inline constexpr char CHASSIS_TARGET_TOPIC[] = "nav_data";
inline constexpr char DECISION_COMMAND_TOPIC[] = "behavior_data";
inline constexpr char TEAM_INFO_TOPIC[] = "team_info";
inline constexpr char GAME_INFO_TOPIC[] = "game_info";
inline constexpr char ONLINE_INFO_TOPIC[] = "online_info";
inline constexpr char OFFLINE_INFO_TOPIC[] = "offline_info";
inline constexpr char RADAR_INFO_TOPIC[] = "radar_info";
inline constexpr char GIMBAL_FEEDBACK_TOPIC[] = "nav_gimbal_feedback";
inline constexpr char LAUNCHER_FEEDBACK_TOPIC[] = "nav_launcher_feedback_v1";

inline constexpr uint8_t FEEDBACK_YAW_VALID = 1U << 0U;
inline constexpr uint8_t FEEDBACK_PITCH_VALID = 1U << 1U;
inline constexpr uint8_t FEEDBACK_ANGULAR_VELOCITY_VALID = 1U << 2U;
inline constexpr uint8_t FEEDBACK_BULLET_SPEED_VALID = 1U << 3U;
inline constexpr uint8_t FEEDBACK_BULLET_COUNT_VALID = 1U << 4U;
inline constexpr uint8_t FEEDBACK_GIMBAL_MODE_VALID = 1U << 5U;
inline constexpr uint8_t FEEDBACK_VISION_TASK_VALID = 1U << 6U;
inline constexpr uint8_t FEEDBACK_VALID_MASK = 0x7FU;
inline constexpr uint8_t GIMBAL_BULLET_SPEED_VALID =
    FEEDBACK_BULLET_SPEED_VALID;
inline constexpr uint8_t GIMBAL_BULLET_COUNT_VALID =
    FEEDBACK_BULLET_COUNT_VALID;
inline constexpr uint8_t GIMBAL_MODE_VALID = FEEDBACK_GIMBAL_MODE_VALID;
inline constexpr uint8_t GIMBAL_SHOOT_MODE_VALID = FEEDBACK_VISION_TASK_VALID;

// Compact launcher sample consumed by HostDataLink before publishing the
// aggregate 28-byte GimbalFeedback payload.
struct [[gnu::packed]] GimbalFeedbackV1 {
  float bullet_speed_mps{};
  uint16_t bullet_count{};
  uint8_t gimbal_mode{};
  uint8_t shoot_mode{};
  uint8_t valid_flags{};
  uint8_t reserved[3]{};
};

inline constexpr uint16_t REF_CMD_GAME_STATUS = 0x0001U;
inline constexpr uint16_t REF_CMD_GAME_ROBOT_HP = 0x0003U;
inline constexpr uint16_t REF_CMD_FIELD_EVENTS = 0x0101U;
inline constexpr uint16_t REF_CMD_ROBOT_STATUS = 0x0201U;
inline constexpr uint16_t REF_CMD_POWER_HEAT = 0x0202U;
inline constexpr uint16_t REF_CMD_ROBOT_POSITION = 0x0203U;
inline constexpr uint16_t REF_CMD_ROBOT_BUFF = 0x0204U;
inline constexpr uint16_t REF_CMD_ROBOT_DAMAGE = 0x0206U;
inline constexpr uint16_t REF_CMD_BULLET_REMAINING = 0x0208U;
inline constexpr uint16_t REF_CMD_RFID = 0x0209U;
inline constexpr uint16_t REF_CMD_GROUND_ROBOT_POSITION = 0x020BU;
inline constexpr uint16_t REF_CMD_SENTRY_INFO = 0x020DU;

inline constexpr uint16_t SOURCE_GAME_STATUS = 1U << 0U;
inline constexpr uint16_t SOURCE_ROBOT_HP = 1U << 1U;
inline constexpr uint16_t SOURCE_FIELD_EVENT = 1U << 2U;
inline constexpr uint16_t SOURCE_ROBOT_STATUS = 1U << 3U;
inline constexpr uint16_t SOURCE_POWER_HEAT = 1U << 4U;
inline constexpr uint16_t SOURCE_ROBOT_POSITION = 1U << 5U;
inline constexpr uint16_t SOURCE_ROBOT_BUFF = 1U << 6U;
inline constexpr uint16_t SOURCE_ROBOT_DAMAGE = 1U << 7U;
inline constexpr uint16_t SOURCE_BULLET_REMAINING = 1U << 8U;
inline constexpr uint16_t SOURCE_RFID = 1U << 9U;
inline constexpr uint16_t SOURCE_SENTRY_POS = 1U << 10U;
inline constexpr uint16_t SUPPORTED_REFEREE_SOURCE_MASK =
    SOURCE_GAME_STATUS | SOURCE_ROBOT_HP | SOURCE_FIELD_EVENT |
    SOURCE_ROBOT_STATUS | SOURCE_POWER_HEAT | SOURCE_ROBOT_POSITION |
    SOURCE_ROBOT_BUFF | SOURCE_ROBOT_DAMAGE | SOURCE_BULLET_REMAINING |
    SOURCE_RFID | SOURCE_SENTRY_POS;
inline constexpr uint16_t REQUIRED_REFEREE_SOURCE_MASK =
    SOURCE_GAME_STATUS | SOURCE_ROBOT_HP | SOURCE_FIELD_EVENT |
    SOURCE_ROBOT_STATUS | SOURCE_POWER_HEAT | SOURCE_ROBOT_BUFF |
    SOURCE_BULLET_REMAINING | SOURCE_RFID | SOURCE_SENTRY_POS;

// ROS2_LIBXR wire payloads. Field order and one-byte packing are part of the
// external contract; do not reorder or substitute integer widths.
struct [[gnu::packed]] ChassisTarget {
  float vx_mps{};      /* chassis-frame velocity, m/s */
  float vy_mps{};      /* chassis-frame velocity, m/s */
  float wz_rad_s{};    /* yaw rate, rad/s */
  float current_yaw{}; /* legacy telemetry; unused by this receiver */
  float current_vx{};
  float current_vy{};
  float current_wz{};
  float fx{}; /* legacy force fields must be zero */
  float fy{};
  float mz{};
  bool use_speed_control{};
  float delta_yaw{};
};

struct [[gnu::packed]] BehaviorData {
  uint8_t pitch_mode{};
  uint8_t desire_stance{};
  uint8_t desire_lifter_pos{};
  float scan_yaw_min_rad{}; /* rad */
  float scan_yaw_max_rad{}; /* rad */
  uint16_t ammo_purchase_request{};
  uint8_t revive_request{};
  uint8_t remote_revive_request{};
  uint8_t remote_ammo_request{};
  uint8_t remote_health_request{};
  bool use_limited_scan{};
  bool not_aim_enemy{};
  bool use_capacitor{};
  bool tunnel_align_active{};
  float tunnel_align_angle_rad{}; /* rad */
  bool use_gyro_mode{};
};

struct [[gnu::packed]] TeamInfo {
  struct [[gnu::packed]] AllyRobotStatus {
    uint8_t robot_id{};
    uint16_t robot_hp{};
    float robot_pos_x{};
    float robot_pos_y{};
  };
  AllyRobotStatus ally_status[4]{};
  uint16_t outpost_hp{};
  uint16_t base_hp{};
};

struct [[gnu::packed]] GameInfo {
  uint16_t game_time_remaining{};
  uint16_t coin_remaining{};
  uint32_t event_code{};
  uint8_t game_status{};
  float manual_point_x{};
  float manual_point_y{};
  uint8_t manual_key{};
  uint16_t enemy_outpost_hp{};
  uint16_t enemy_base_hp{};
};

struct [[gnu::packed]] SentryInfoOnline {
  uint16_t self_health{};
  uint16_t bullets_remaining{};
  uint16_t cooling_value{};
  uint16_t heat_limit{};
  uint16_t current_heat{};
  float sentry_pos_x{};
  float sentry_pos_y{};
  float speed_monitor_angle{};
  uint32_t sentry_info_1{};
  uint16_t sentry_info_2{};
  uint64_t sentry_info_3{};
  uint8_t energy_ratio{};
};

struct [[gnu::packed]] SentryInfoOffline {
  float yaw_camerainit_to_gimbal{}; /* deg, counterclockwise positive */
  uint8_t lifter_current_pos{};
  bool is_transformable{};
  float transform_state{};
  uint8_t capacitor_capacity{}; /* 0..100 percent; 255 means unavailable */
  float chassis_imu_yaw{};      /* rad */
  // True when the active tunnel-alignment request is within the firmware
  // alignment tolerance.
  bool tunnel_yaw_aligned{};
};

struct [[gnu::packed]] EnemyRobotStatus {
  uint8_t robot_id{};
  uint16_t robot_hp{};
  uint16_t allowed_projectile{};
  uint16_t robot_pos_x{};
  uint16_t robot_pos_y{};
};

struct [[gnu::packed]] RadarInfo {
  EnemyRobotStatus enemy_status[6]{};
  uint16_t enemy_coin_left{};
  uint16_t enemy_coin_accumulated{};
  bool is_enemy_outpost_sensed{};
};

struct [[gnu::packed]] GimbalFeedback {
  float yaw{};
  float pitch{};
  float yaw_velocity{};
  float pitch_velocity{};
  float bullet_speed{};
  uint16_t bullet_count{};
  uint8_t gimbal_mode{};
  uint8_t vision_task{};
  uint8_t valid_flags{};
  uint8_t reserved[3]{};
};

static_assert(std::endian::native == std::endian::little);
static_assert(sizeof(ChassisTarget) == 45U);
static_assert(sizeof(BehaviorData) == 26U);
static_assert(sizeof(TeamInfo::AllyRobotStatus) == 11U);
static_assert(sizeof(TeamInfo) == 48U);
static_assert(sizeof(GameInfo) == 22U);
static_assert(sizeof(SentryInfoOnline) == 37U);
static_assert(sizeof(SentryInfoOffline) == 16U);
static_assert(sizeof(EnemyRobotStatus) == 9U);
static_assert(sizeof(RadarInfo) == 59U);
static_assert(sizeof(GimbalFeedback) == 28U);
static_assert(sizeof(GimbalFeedbackV1) == 12U);
static_assert(offsetof(ChassisTarget, wz_rad_s) == 8U);
static_assert(offsetof(ChassisTarget, use_speed_control) == 40U);
static_assert(offsetof(ChassisTarget, delta_yaw) == 41U);
static_assert(offsetof(BehaviorData, ammo_purchase_request) == 11U);
static_assert(offsetof(BehaviorData, use_gyro_mode) == 25U);
static_assert(offsetof(SentryInfoOffline, yaw_camerainit_to_gimbal) == 0U);
static_assert(offsetof(SentryInfoOffline, lifter_current_pos) == 4U);
static_assert(offsetof(SentryInfoOffline, is_transformable) == 5U);
static_assert(offsetof(SentryInfoOffline, transform_state) == 6U);
static_assert(offsetof(SentryInfoOffline, capacitor_capacity) == 10U);
static_assert(offsetof(SentryInfoOffline, chassis_imu_yaw) == 11U);
static_assert(offsetof(SentryInfoOffline, tunnel_yaw_aligned) == 15U);
static_assert(offsetof(TeamInfo, outpost_hp) == 44U);
static_assert(std::is_trivially_copyable_v<ChassisTarget>);
static_assert(std::is_trivially_copyable_v<BehaviorData>);
static_assert(std::is_trivially_copyable_v<TeamInfo>);
static_assert(std::is_trivially_copyable_v<GameInfo>);
static_assert(std::is_trivially_copyable_v<SentryInfoOnline>);
static_assert(std::is_trivially_copyable_v<SentryInfoOffline>);
static_assert(std::is_trivially_copyable_v<RadarInfo>);

}  // namespace Pldx::HostDataLink

namespace Pldx::HostDataLinkDetail {

inline constexpr uint32_t COMMAND_TIMEOUT_MS = 150U;
inline constexpr float CHASSIS_YAW_ALIGNMENT_TOLERANCE_RAD = 0.05F;
inline constexpr uint32_t AUTO_CHASSIS_MODE_ROTOR = 6U;
inline constexpr uint32_t AUTO_CHASSIS_MODE_NAVIGATION = 4U;
inline constexpr uint32_t FEEDBACK_OMEGA_TIMEOUT_MS = 50U;
inline constexpr uint32_t FEEDBACK_FIELD_TIMEOUT_MS = 250U;

inline bool AngularVelocityFresh(LibXR::MillisecondTimestamp now,
                                 LibXR::MillisecondTimestamp yaw_time,
                                 LibXR::MillisecondTimestamp pitch_time,
                                 bool yaw_seen, bool pitch_seen) {
  return yaw_seen && pitch_seen &&
         (now - yaw_time).ToMillisecond() <= FEEDBACK_OMEGA_TIMEOUT_MS &&
         (now - pitch_time).ToMillisecond() <= FEEDBACK_OMEGA_TIMEOUT_MS;
}

inline float YawRadToCamerainitDeg(float yaw_rad) {
  if (!std::isfinite(yaw_rad)) {
    return 0.0F;
  }
  const float DEG = yaw_rad * (180.0F / static_cast<float>(LibXR::PI));
  float wrapped = std::remainder(DEG, 360.0F);
  if (wrapped <= -180.0F) {
    wrapped = 180.0F;
  }
  return wrapped;
}

// Validate wire bool representations before copying bytes into C++ bools.
inline bool ChassisPayloadValid(LibXR::ConstRawData payload) {
  return payload.addr_ != nullptr &&
         payload.size_ == sizeof(HostDataLink::ChassisTarget) &&
         static_cast<const uint8_t*>(payload.addr_)[40] <= 1U;
}

inline bool BehaviorPayloadValid(LibXR::ConstRawData payload) {
  if (payload.addr_ == nullptr ||
      payload.size_ != sizeof(HostDataLink::BehaviorData))
    return false;
  const auto* bytes = static_cast<const uint8_t*>(payload.addr_);
  return bytes[17] <= 1U && bytes[18] <= 1U && bytes[19] <= 1U &&
         bytes[20] <= 1U && bytes[25] <= 1U;
}

inline bool IsTunnelYawAligned(float chassis_yaw_rad, bool align_active,
                               float target_yaw_rad, bool yaw_valid) {
  if (!align_active || !yaw_valid || !std::isfinite(chassis_yaw_rad) ||
      !std::isfinite(target_yaw_rad)) {
    return false;
  }
  const float error = std::remainder(chassis_yaw_rad - target_yaw_rad,
                                     static_cast<float>(LibXR::TWO_PI));
  return std::fabs(error) <= CHASSIS_YAW_ALIGNMENT_TOLERANCE_RAD;
}

inline bool ChassisCommandValid(const HostDataLink::ChassisTarget& command) {
  // The inter-board navigation format represents signed velocity in 0.001
  // units.
  constexpr float SANITY_LIMIT = 32.767F;
  for (float value : {command.vx_mps, command.vy_mps, command.wz_rad_s}) {
    if (!std::isfinite(value) || std::fabs(value) > SANITY_LIMIT) return false;
  }
  return command.use_speed_control && std::isfinite(command.current_yaw) &&
         std::isfinite(command.current_vx) &&
         std::isfinite(command.current_vy) &&
         std::isfinite(command.current_wz) &&
         std::isfinite(command.delta_yaw) && command.fx == 0.0F &&
         command.fy == 0.0F && command.mz == 0.0F;
}

class ChassisCommandTracker {
 public:
  bool Observe(const HostDataLink::ChassisTarget& command,
               LibXR::MillisecondTimestamp now) {
    Expire(now);
    command_seen_ = true;
    finite_ = ChassisCommandValid(command);
    if (!finite_) return Reject();
    Accept(now);
    return true;
  }

  bool CommandFresh(LibXR::MillisecondTimestamp now) {
    Expire(now);
    return armed_ && accepted_command_seen_;
  }
  uint32_t StatusFlags(LibXR::MillisecondTimestamp now) {
    uint32_t flags = 1U;
    if (command_seen_) flags |= 1U << 1U;
    if (CommandFresh(now)) flags |= 1U << 2U;
    if (finite_) flags |= 1U << 4U;
    return flags;
  }
  uint32_t CommandAge(LibXR::MillisecondTimestamp now) const {
    return accepted_command_seen_ ? (now - last_command_time_).ToMillisecond()
                                  : std::numeric_limits<uint32_t>::max();
  }
  uint32_t LastSequence() const { return accepted_count_; }
  uint32_t RejectedCount() const { return rejected_count_; }

 private:
  void Accept(LibXR::MillisecondTimestamp now) {
    last_command_time_ = now;
    accepted_command_seen_ = true;
    armed_ = true;
    ++accepted_count_;
  }
  bool Reject() {
    armed_ = false;
    ++rejected_count_;
    return false;
  }
  void Expire(LibXR::MillisecondTimestamp now) {
    if (armed_ &&
        (now - last_command_time_).ToMillisecond() >= COMMAND_TIMEOUT_MS) {
      armed_ = false;
    }
  }
  LibXR::MillisecondTimestamp last_command_time_{};
  uint32_t rejected_count_{}, accepted_count_{};
  bool command_seen_{}, accepted_command_seen_{}, finite_{};
  bool armed_{};
};

struct DecisionPublication {
  bool accepted = false;
  bool publish_buy_bullet_num = false;
  bool publish_remote_buy_bullet_times = false;
  bool publish_remote_buy_hp_times = false;
  bool publish_buy_resurrection = false;
  uint16_t buy_bullet_num = 0U;
  uint8_t remote_buy_bullet_times = 0U;
  uint8_t remote_buy_hp_times = 0U;
  bool buy_resurrection = false;
};

class DecisionSequenceTracker {
 public:
  // BehaviorData uses independent monotonic counters; a decrease resets that
  // counter's baseline and emits no event for that counter.
  DecisionPublication Decode(const HostDataLink::BehaviorData& value,
                             bool ready = true) {
    DecisionPublication result;
    if (!ready || !std::isfinite(value.scan_yaw_min_rad) ||
        !std::isfinite(value.scan_yaw_max_rad) ||
        !std::isfinite(value.tunnel_align_angle_rad)) {
      ++rejected_count_;
      return result;
    }
    result.accepted = true;
    if (!seen_) {
      seen_ = true;
    } else {
      result.buy_bullet_num = Delta(value.ammo_purchase_request, ammo_);
      result.remote_buy_bullet_times =
          Delta(value.remote_ammo_request, remote_ammo_);
      result.remote_buy_hp_times =
          Delta(value.remote_health_request, remote_hp_);
      const bool ammo_rollback = value.ammo_purchase_request < ammo_;
      const bool remote_ammo_rollback =
          value.remote_ammo_request < remote_ammo_;
      const bool remote_hp_rollback = value.remote_health_request < remote_hp_;
      const bool revive_rollback = value.revive_request < revive_ ||
                                   value.remote_revive_request < remote_revive_;
      result.publish_buy_bullet_num =
          !ammo_rollback && result.buy_bullet_num != 0U;
      result.publish_remote_buy_bullet_times =
          !remote_ammo_rollback && result.remote_buy_bullet_times != 0U;
      result.publish_remote_buy_hp_times =
          !remote_hp_rollback && result.remote_buy_hp_times != 0U;
      result.publish_buy_resurrection =
          !revive_rollback && (value.revive_request > revive_ ||
                               value.remote_revive_request > remote_revive_);
      result.buy_resurrection = result.publish_buy_resurrection;
    }
    ammo_ = value.ammo_purchase_request;
    remote_ammo_ = value.remote_ammo_request;
    remote_hp_ = value.remote_health_request;
    revive_ = value.revive_request;
    remote_revive_ = value.remote_revive_request;
    return result;
  }
  uint32_t RejectedCount() const { return rejected_count_; }

 private:
  template <typename T>
  static T Delta(T value, T previous) {
    return value >= previous ? static_cast<T>(value - previous) : T{};
  }
  uint16_t ammo_{};
  uint8_t remote_ammo_{}, remote_hp_{}, revive_{}, remote_revive_{};
  uint32_t rejected_count_{};
  bool seen_{};
};

template <typename Sink>
bool DispatchDecision(const DecisionPublication& value, Sink& sink) {
  if (!value.accepted) return false;
  if (value.publish_buy_bullet_num)
    sink.PublishBuyBulletNum(value.buy_bullet_num);
  if (value.publish_remote_buy_bullet_times)
    sink.PublishRemoteBuyBulletTimes(value.remote_buy_bullet_times);
  if (value.publish_remote_buy_hp_times)
    sink.PublishRemoteBuyHpTimes(value.remote_buy_hp_times);
  if (value.publish_buy_resurrection)
    sink.PublishBuyResurrection(value.buy_resurrection);
  return true;
}

}  // namespace Pldx::HostDataLinkDetail

#include "CMD.hpp"
#include "Referee.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "timebase.hpp"
#include "timer.hpp"

namespace Pldx::HostDataLinkDetail {
inline CMD::Data MakeAIChassis(const HostDataLink::ChassisTarget& command,
                               bool fresh) {
  CMD::Data value{};
  value.ctrl_source = CMD::ControlSource::CTRL_SOURCE_AI;
  if (fresh) {
    value.chassis.vx = command.vx_mps;
    value.chassis.vy = command.vy_mps;
    value.chassis.wz = command.wz_rad_s;
    value.chassis_online = true;
  }
  return value;
}
}  // namespace Pldx::HostDataLinkDetail

class NavLink {
 public:
  /// @brief 监控循环入口，由静态装配的监控循环调用。
  void OnMonitor() {}

  NavLink(CMD& cmd, const char* referee_topic_name = "sentry_ref",
               const char* pitch_topic_name = "pitchmotor_angle",
               const char* yaw_topic_name = "yawmotor_angle",
               const char* quaternion_topic_name = "ahrs_quaternion");

 private:
  static LibXR::MillisecondTimestamp Now() {
    return LibXR::Timebase::GetMilliseconds();
  }
  template <typename Data, void (NavLink::*Method)(const Data&)>
  void RegisterCallback(LibXR::Topic& topic) {
    auto cb = LibXR::Topic::Callback::Create(
        [](bool in_isr, NavLink* self,
           const LibXR::Topic::RawMessageView& message) {
          ASSERT(!in_isr);
          if (in_isr) return;
          if (message.payload.addr_ == nullptr ||
              message.payload.size_ != sizeof(Data))
            return;
          Data data{};
          LibXR::Memory::FastCopy(&data, message.payload.addr_, sizeof(data));
          (self->*Method)(data);
        },
        this);
    topic.RegisterCallback(cb);
  }
  template <
      void (NavLink::*Method)(const Pldx::HostDataLink::ChassisTarget&)>
  void RegisterRawCallback(LibXR::Topic& topic) {
    auto cb = LibXR::Topic::Callback::Create(
        [](bool in_isr, NavLink* self,
           const LibXR::Topic::RawMessageView& message) {
          ASSERT(!in_isr);
          if (in_isr) return;
          if (message.timestamp == 0U ||
              !Pldx::HostDataLinkDetail::ChassisPayloadValid(message.payload)) {
            return;
          }
          Pldx::HostDataLink::ChassisTarget value{};
          LibXR::Memory::FastCopy(&value, message.payload.addr_, sizeof(value));
          (self->*Method)(value);
        },
        this);
    topic.RegisterCallback(cb);
  }
  template <
      void (NavLink::*Method)(const Pldx::HostDataLink::BehaviorData&)>
  void RegisterRawCallback(LibXR::Topic& topic) {
    auto cb = LibXR::Topic::Callback::Create(
        [](bool in_isr, NavLink* self,
           const LibXR::Topic::RawMessageView& message) {
          ASSERT(!in_isr);
          if (in_isr) return;
          if (message.timestamp == 0U ||
              !Pldx::HostDataLinkDetail::BehaviorPayloadValid(
                  message.payload)) {
            return;
          }
          Pldx::HostDataLink::BehaviorData value{};
          LibXR::Memory::FastCopy(&value, message.payload.addr_, sizeof(value));
          (self->*Method)(value);
        },
        this);
    topic.RegisterCallback(cb);
  }
  template <typename Data, void (NavLink::*Method)(const Data&)>
  void RegisterNamedCallback(const char* name) {
    LibXR::Topic topic(LibXR::Topic::FindOrCreate<Data>(name));
    RegisterCallback<Data, Method>(topic);
  }
  void OnChassis(const Pldx::HostDataLink::ChassisTarget& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    const auto NOW = Now();
    if (tracker_.Observe(value, NOW)) {
      latest_chassis_ = value;
      chassis_submitted_online_ = true;
      cmd_->FeedAIChassis(BuildAI(NOW));
    } else if (chassis_submitted_online_) {
      chassis_submitted_online_ = false;
      cmd_->FeedAIChassis(BuildAI(NOW));
    }
  }
  void CheckChassisTimeout() {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (chassis_submitted_online_ && !tracker_.CommandFresh(Now())) {
      chassis_submitted_online_ = false;
      cmd_->FeedAIChassis(BuildAI(Now()));
    }
    const auto now = Now();
    bool changed = false;
    if (chassis_imu_yaw_valid_ &&
        (now - chassis_yaw_time_).ToMillisecond() > 250U) {
      chassis_imu_yaw_valid_ = false;
      changed = true;
    }
    if (capacitor_capacity_valid_ &&
        (now - capacitor_time_).ToMillisecond() > 300U) {
      capacitor_capacity_valid_ = false;
      changed = true;
    }
    if (tunnel_align_active_ && (now - behavior_time_).ToMillisecond() > 250U) {
      tunnel_align_active_ = false;
      changed = true;
    }
    if (changed) PublishOffline();
    if (RefreshGimbalFeedback(Now()))
      gimbal_feedback_topic_.Publish(gimbal_feedback_);
  }
  void OnBehavior(const Pldx::HostDataLink::BehaviorData& value) {
    Pldx::HostDataLinkDetail::DecisionPublication publication;
    uint32_t chassis_mode =
        Pldx::HostDataLinkDetail::AUTO_CHASSIS_MODE_NAVIGATION;
    {
      LibXR::Mutex::LockGuard lock(mutex_);
      publication = decision_.Decode(value);
      if (!publication.accepted) return;
      tunnel_align_active_ = value.tunnel_align_active;
      tunnel_align_angle_rad_ = value.tunnel_align_angle_rad;
      behavior_time_ = Now();
      PublishOffline();
      chassis_mode =
          value.use_gyro_mode
              ? Pldx::HostDataLinkDetail::AUTO_CHASSIS_MODE_ROTOR
              : Pldx::HostDataLinkDetail::AUTO_CHASSIS_MODE_NAVIGATION;
    }
    bool use_capacitor = value.use_capacitor;
    use_capacitor_topic_.Publish(use_capacitor);
    nav_chassis_mode_topic_.Publish(chassis_mode);
    Sink sink(*this);
    Pldx::HostDataLinkDetail::DispatchDecision(publication, sink);
  }
  void OnLauncherFeedback(const Pldx::HostDataLink::GimbalFeedbackV1& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if ((value.valid_flags &
         static_cast<uint8_t>(~Pldx::HostDataLink::FEEDBACK_VALID_MASK)) != 0U)
      return;
    gimbal_feedback_.bullet_speed = value.bullet_speed_mps;
    gimbal_feedback_.bullet_count = value.bullet_count;
    constexpr uint8_t kLauncherFlags =
        Pldx::HostDataLink::FEEDBACK_BULLET_SPEED_VALID |
        Pldx::HostDataLink::FEEDBACK_BULLET_COUNT_VALID;
    gimbal_feedback_.valid_flags =
        static_cast<uint8_t>((gimbal_feedback_.valid_flags & ~kLauncherFlags) |
                             (value.valid_flags & kLauncherFlags));
    launcher_valid_flags_ =
        static_cast<uint8_t>(value.valid_flags & kLauncherFlags);
    launcher_feedback_seen_ = true;
    launcher_feedback_time_ = Now();
    PublishGimbalFeedback();
  }
  void OnChassisYaw(const float& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    chassis_imu_yaw_ = value;
    chassis_yaw_time_ = Now();
    PublishOffline();
  }
  void OnChassisYawValid(const bool& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    chassis_imu_yaw_valid_ = value;
    PublishOffline();
  }
  void OnCapacitorCapacity(const uint8_t& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (value <= 100U) {
      capacitor_capacity_ = value;
      capacitor_time_ = Now();
    } else {
      capacitor_capacity_valid_ = false;
    }
    PublishOffline();
  }
  void OnCapacitorValid(const bool& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    capacitor_capacity_valid_ = value;
    PublishOffline();
  }
  void OnGimbalYaw(const float& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (!std::isfinite(value)) return;
    yawmotor_angle_ = value;
    gimbal_feedback_.yaw = value;
    yaw_feedback_time_ = Now();
    yaw_feedback_seen_ = true;
    gimbal_feedback_.valid_flags |= Pldx::HostDataLink::FEEDBACK_YAW_VALID;
    PublishGimbalFeedback();
    PublishOffline();
  }
  void OnGimbalPitch(const float& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (!std::isfinite(value)) return;
    gimbal_feedback_.pitch = value;
    pitch_feedback_time_ = Now();
    pitch_feedback_seen_ = true;
    gimbal_feedback_.valid_flags |= Pldx::HostDataLink::FEEDBACK_PITCH_VALID;
    PublishGimbalFeedback();
  }
  void OnGimbalYawOmega(const float& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (!std::isfinite(value)) return;
    gimbal_feedback_.yaw_velocity = value;
    yaw_omega_time_ = Now();
    yaw_omega_seen_ = true;
    PublishGimbalFeedback();
  }
  void OnGimbalPitchOmega(const float& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    if (!std::isfinite(value)) return;
    gimbal_feedback_.pitch_velocity = value;
    pitch_omega_time_ = Now();
    pitch_omega_seen_ = true;
    PublishGimbalFeedback();
  }
  void OnGimbalMode(const uint8_t& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    gimbal_feedback_.gimbal_mode = value;
    mode_feedback_time_ = Now();
    mode_feedback_seen_ = true;
    gimbal_feedback_.valid_flags |=
        Pldx::HostDataLink::FEEDBACK_GIMBAL_MODE_VALID;
    PublishGimbalFeedback();
  }
  void OnVisionTask(const uint8_t& value) {
    LibXR::Mutex::LockGuard lock(mutex_);
    gimbal_feedback_.vision_task = value;
    vision_feedback_time_ = Now();
    vision_feedback_seen_ = true;
    gimbal_feedback_.valid_flags |=
        Pldx::HostDataLink::FEEDBACK_VISION_TASK_VALID;
    PublishGimbalFeedback();
  }
  void PublishGimbalFeedback() {
    const auto NOW = Now();
    RefreshGimbalFeedback(NOW);
    gimbal_feedback_topic_.Publish(gimbal_feedback_);
  }
  bool RefreshGimbalFeedback(LibXR::MillisecondTimestamp now) {
    const uint8_t previous = gimbal_feedback_.valid_flags;
    uint8_t flags = 0U;
    const auto fresh = [now](LibXR::MillisecondTimestamp seen, bool has_seen) {
      return has_seen &&
             (now - seen).ToMillisecond() <=
                 Pldx::HostDataLinkDetail::FEEDBACK_FIELD_TIMEOUT_MS;
    };
    if (fresh(yaw_feedback_time_, yaw_feedback_seen_))
      flags |= Pldx::HostDataLink::FEEDBACK_YAW_VALID;
    if (fresh(pitch_feedback_time_, pitch_feedback_seen_))
      flags |= Pldx::HostDataLink::FEEDBACK_PITCH_VALID;
    if (fresh(mode_feedback_time_, mode_feedback_seen_))
      flags |= Pldx::HostDataLink::FEEDBACK_GIMBAL_MODE_VALID;
    if (fresh(vision_feedback_time_, vision_feedback_seen_))
      flags |= Pldx::HostDataLink::FEEDBACK_VISION_TASK_VALID;
    if (fresh(launcher_feedback_time_, launcher_feedback_seen_))
      flags |= launcher_valid_flags_;
    if (Pldx::HostDataLinkDetail::AngularVelocityFresh(
            now, yaw_omega_time_, pitch_omega_time_, yaw_omega_seen_,
            pitch_omega_seen_) &&
        std::isfinite(gimbal_feedback_.yaw_velocity) &&
        std::isfinite(gimbal_feedback_.pitch_velocity)) {
      flags |= Pldx::HostDataLink::FEEDBACK_ANGULAR_VELOCITY_VALID;
    }
    gimbal_feedback_.valid_flags = flags;
    return flags != previous;
  }
  void PublishOffline() {
    Pldx::HostDataLink::SentryInfoOffline info{};
    info.yaw_camerainit_to_gimbal =
        Pldx::HostDataLinkDetail::YawRadToCamerainitDeg(yawmotor_angle_);
    info.lifter_current_pos = 0U;
    info.is_transformable = false;
    info.transform_state = 0.0F;
    info.chassis_imu_yaw = chassis_imu_yaw_valid_
                               ? chassis_imu_yaw_
                               : std::numeric_limits<float>::quiet_NaN();
    // 255 is the ABI-preserving unknown sentinel; 0..100 are percent.
    info.capacitor_capacity =
        capacitor_capacity_valid_ ? capacitor_capacity_ : 255U;
    info.tunnel_yaw_aligned = Pldx::HostDataLinkDetail::IsTunnelYawAligned(
        chassis_imu_yaw_, tunnel_align_active_, tunnel_align_angle_rad_,
        chassis_imu_yaw_valid_);
    offline_info_topic_.Publish(info);
  }
  CMD::Data BuildAI(LibXR::MillisecondTimestamp now) {
    return Pldx::HostDataLinkDetail::MakeAIChassis(latest_chassis_,
                                                   tracker_.CommandFresh(now));
  }
  void OnReferee(const Referee::RobotGameRefereePack& source) {
    const auto fresh = [&source](uint16_t required) {
      return source.referee_online &&
             (source.source_fresh_mask & required) == required;
    };
    Pldx::HostDataLink::GameInfo game{};
    game.game_time_remaining = source.game_status.stage_remain_time;
    game.coin_remaining = source.bullet_remain.coin_remain;
    game.game_status = static_cast<uint8_t>(source.game_status.game_progress);
    LibXR::Memory::FastCopy(&game.event_code, &source.field_event,
                            sizeof(game.event_code));
    game.enemy_outpost_hp = source.robot_hp.enemy_outpost_hp;
    game.enemy_base_hp = source.robot_hp.enemy_base_hp;
    if (fresh(Referee::SOURCE_GAME_STATUS | Referee::SOURCE_BULLET_REMAIN |
              Referee::SOURCE_FIELD_EVENT | Referee::SOURCE_ROBOT_HP))
      game_info_topic_.Publish(game);

    Pldx::HostDataLink::SentryInfoOnline online{};
    online.self_health = source.robot_status.remain_hp;
    online.bullets_remaining = source.bullet_remain.bullet_17_remain;
    online.cooling_value = source.robot_status.shooter_cooling_value;
    online.heat_limit = source.robot_status.shooter_heat_limit;
    online.current_heat = source.power_heat.launcher_id1_17_heat;
    online.sentry_pos_x = source.robot_pos.x;
    online.sentry_pos_y = source.robot_pos.y;
    online.speed_monitor_angle = source.robot_pos.angle;
    const auto& sentry = source.sentry_info;
    online.sentry_info_1 = (sentry.exchanged_bullet_num & 0x7FFU) |
                           ((sentry.exchanged_bullet_times & 0x0FU) << 11U) |
                           ((sentry.exchanged_blood_times & 0x0FU) << 15U) |
                           ((sentry.could_risen_free & 0x01U) << 19U) |
                           ((sentry.could_risen_exchanged & 0x01U) << 20U) |
                           ((sentry.risen_cost & 0x03FFU) << 21U);
    online.sentry_info_2 =
        static_cast<uint16_t>((sentry.current_state & 0x03U) |
                              ((sentry.own_mech_state & 0x01U) << 2U));
    // info_3 posture-buff timers are not present in the referee source.
    online.energy_ratio = source.robot_buff.percent_100_remain_energy  ? 100U
                          : source.robot_buff.percent_50_remain_energy ? 50U
                                                                       : 0U;
    if (fresh(Referee::SOURCE_ROBOT_STATUS | Referee::SOURCE_BULLET_REMAIN |
              Referee::SOURCE_POWER_HEAT | Referee::SOURCE_ROBOT_POS |
              Referee::SOURCE_SENTRY_INFO | Referee::SOURCE_ROBOT_BUFF))
      online_info_topic_.Publish(online);

    const uint8_t id = source.robot_status.robot_id;
    const bool blue = id >= 101U && id <= 199U;
    const bool ids_valid = blue || (id >= 1U && id <= 99U);
    if (fresh(Referee::SOURCE_ROBOT_HP | Referee::SOURCE_SENTRY_POS |
              Referee::SOURCE_ROBOT_STATUS) &&
        ids_valid) {
      Pldx::HostDataLink::TeamInfo info{};
      const uint8_t ids[4] = {static_cast<uint8_t>(blue ? 101U : 1U),
                              static_cast<uint8_t>(blue ? 102U : 2U),
                              static_cast<uint8_t>(blue ? 103U : 3U),
                              static_cast<uint8_t>(blue ? 104U : 4U)};
      const uint16_t hp[4] = {
          source.robot_hp.ally_1_robot_hp, source.robot_hp.ally_2_robot_hp,
          source.robot_hp.ally_3_robot_hp, source.robot_hp.ally_4_robot_hp};
      const float xy[8] = {
          source.sentry_pos.hero_x,       source.sentry_pos.hero_y,
          source.sentry_pos.engineer_x,   source.sentry_pos.engineer_y,
          source.sentry_pos.standard_3_x, source.sentry_pos.standard_3_y,
          source.sentry_pos.standard_4_x, source.sentry_pos.standard_4_y};
      for (size_t i = 0; i < 4; ++i)
        info.ally_status[i] = {ids[i], hp[i], xy[2 * i], xy[2 * i + 1]};
      info.outpost_hp = source.robot_hp.ally_outpost_hp;
      info.base_hp = source.robot_hp.ally_base_hp;
      team_info_topic_.Publish(info);
    }
  }
  class Sink {
   public:
    explicit Sink(NavLink& owner) : owner_(owner) {}
    void PublishBuyBulletNum(uint16_t v) {
      owner_.buy_bullet_topic_.Publish(v);
    }
    void PublishRemoteBuyBulletTimes(uint8_t v) {
      owner_.remote_bullet_topic_.Publish(v);
    }
    void PublishRemoteBuyHpTimes(uint8_t v) {
      owner_.remote_hp_topic_.Publish(v);
    }
    void PublishBuyResurrection(bool v) {
      owner_.resurrection_topic_.Publish(v);
    }

   private:
    NavLink& owner_;
  };
  CMD* cmd_;
  LibXR::Mutex mutex_;
  LibXR::Topic chassis_topic_, behavior_topic_, team_info_topic_,
      game_info_topic_, online_info_topic_, offline_info_topic_,
      radar_info_topic_, gimbal_feedback_topic_, nav_chassis_mode_topic_,
      use_capacitor_topic_;
  LibXR::Topic buy_bullet_topic_{
      LibXR::Topic::CreateTopic<uint16_t>("sentry_buy_bullet_num")};
  LibXR::Topic remote_bullet_topic_{
      LibXR::Topic::CreateTopic<uint8_t>("sentry_remote_buy_bullet_times")};
  LibXR::Topic remote_hp_topic_{
      LibXR::Topic::CreateTopic<uint8_t>("sentry_remote_buy_hp_times")};
  LibXR::Topic resurrection_topic_{
      LibXR::Topic::CreateTopic<bool>("sentry_buy_resurrection")};
  Pldx::HostDataLink::ChassisTarget latest_chassis_{};
  bool chassis_submitted_online_ = false;
  float yawmotor_angle_{};
  LibXR::MillisecondTimestamp yaw_omega_time_{};
  LibXR::MillisecondTimestamp pitch_omega_time_{};
  bool yaw_omega_seen_{};
  bool pitch_omega_seen_{};
  float chassis_imu_yaw_{};
  bool chassis_imu_yaw_valid_{};
  LibXR::MillisecondTimestamp chassis_yaw_time_{};
  uint8_t capacitor_capacity_{};
  bool capacitor_capacity_valid_{};
  LibXR::MillisecondTimestamp capacitor_time_{};
  bool tunnel_align_active_{};
  LibXR::MillisecondTimestamp behavior_time_{};
  float tunnel_align_angle_rad_{};
  Pldx::HostDataLink::GimbalFeedback gimbal_feedback_{};
  LibXR::MillisecondTimestamp yaw_feedback_time_{}, pitch_feedback_time_{};
  LibXR::MillisecondTimestamp mode_feedback_time_{}, vision_feedback_time_{};
  LibXR::MillisecondTimestamp launcher_feedback_time_{};
  uint8_t launcher_valid_flags_{};
  bool yaw_feedback_seen_{}, pitch_feedback_seen_{}, mode_feedback_seen_{};
  bool vision_feedback_seen_{}, launcher_feedback_seen_{};
  Pldx::HostDataLinkDetail::ChassisCommandTracker tracker_;
  Pldx::HostDataLinkDetail::DecisionSequenceTracker decision_;
};

inline NavLink::HostDataLink(CMD& cmd, const char* referee_topic_name,
                                  const char* pitch_topic_name,
                                  const char* yaw_topic_name,
                                  const char* quaternion_topic_name)
    : cmd_(&cmd),
      chassis_topic_(
          LibXR::Topic::CreateTopic<Pldx::HostDataLink::ChassisTarget>(
              "nav_data")),
      behavior_topic_(
          LibXR::Topic::CreateTopic<Pldx::HostDataLink::BehaviorData>(
              "behavior_data")),
      team_info_topic_(
          LibXR::Topic::CreateTopic<Pldx::HostDataLink::TeamInfo>("team_info")),
      game_info_topic_(
          LibXR::Topic::CreateTopic<Pldx::HostDataLink::GameInfo>("game_info")),
      online_info_topic_(
          LibXR::Topic::CreateTopic<Pldx::HostDataLink::SentryInfoOnline>(
              "online_info")),
      offline_info_topic_(
          LibXR::Topic::CreateTopic<Pldx::HostDataLink::SentryInfoOffline>(
              "offline_info")),
      radar_info_topic_(
          LibXR::Topic::CreateTopic<Pldx::HostDataLink::RadarInfo>(
              "radar_info")),
      gimbal_feedback_topic_(
          LibXR::Topic::CreateTopic<Pldx::HostDataLink::GimbalFeedback>(
              Pldx::HostDataLink::GIMBAL_FEEDBACK_TOPIC)),
      nav_chassis_mode_topic_(LibXR::Topic::CreateTopic<uint32_t>(
          "nav_chassis_mode", nullptr, false)),
      use_capacitor_topic_(
          LibXR::Topic::CreateTopic<bool>("use_capacitor", nullptr, false)) {
  RegisterRawCallback<&NavLink::OnChassis>(chassis_topic_);
  RegisterRawCallback<&NavLink::OnBehavior>(behavior_topic_);
  RegisterNamedCallback<Referee::RobotGameRefereePack,
                        &NavLink::OnReferee>(referee_topic_name);
  RegisterNamedCallback<Pldx::HostDataLink::GimbalFeedbackV1,
                        &NavLink::OnLauncherFeedback>(
      Pldx::HostDataLink::LAUNCHER_FEEDBACK_TOPIC);
  RegisterNamedCallback<float, &NavLink::OnGimbalYaw>(yaw_topic_name);
  RegisterNamedCallback<float, &NavLink::OnGimbalPitch>(pitch_topic_name);
  RegisterNamedCallback<float, &NavLink::OnGimbalYawOmega>(
      "yawmotor_omega");
  RegisterNamedCallback<float, &NavLink::OnGimbalPitchOmega>(
      "pitchmotor_omega");
  RegisterNamedCallback<uint8_t, &NavLink::OnGimbalMode>("gimbal_mode");
  RegisterNamedCallback<uint8_t, &NavLink::OnVisionTask>("vision_task");
  RegisterNamedCallback<float, &NavLink::OnChassisYaw>("chassis_imu_yaw");
  RegisterNamedCallback<bool, &NavLink::OnChassisYawValid>(
      "chassis_imu_yaw_valid");
  RegisterNamedCallback<uint8_t, &NavLink::OnCapacitorCapacity>(
      "chassis_capacitor_capacity");
  RegisterNamedCallback<bool, &NavLink::OnCapacitorValid>(
      "chassis_capacitor_valid");
  auto timeout_timer = LibXR::Timer::CreateTask(
      +[](NavLink* self) { self->CheckChassisTimeout(); }, this, 5);
  LibXR::Timer::Add(timeout_timer);
  LibXR::Timer::Start(timeout_timer);
  UNUSED(pitch_topic_name);
  UNUSED(yaw_topic_name);
  UNUSED(quaternion_topic_name);
}
