#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"


static safety_config volkswagen_meb_init(uint16_t param) {
  // Transmit of GRA_ACC_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MEB_STOCK_TX_MSGS[] = {{MSG_HCA_03, 0, 24, .check_relay = true}, {MSG_GRA_ACC_01, 0, 8, .check_relay = false},
                                                        {MSG_GRA_ACC_01, 2, 8, .check_relay = false}, {MSG_LDW_02, 0, 8, .check_relay = true}};

  static const CanMsg VOLKSWAGEN_MEB_LONG_TX_MSGS[] = {{MSG_HCA_03, 0, 24, .check_relay = true}, {MSG_LDW_02, 0, 8, .check_relay = true},
                                                       {MSG_ACC_18, 0, 32, .check_relay = true}, {MSG_MEB_ACC_01, 0, 48, .check_relay = true}};

  static RxCheck volkswagen_meb_rx_checks[] = {
    {.msg = {{MSG_ESC_51, 0, 48, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_LH_EPS_03, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_QFK_01, 0, 32, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_51, 0, 32, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_14, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_ACC_01, 0, 8, 33U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  volkswagen_common_init();

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#else
  SAFETY_UNUSED(param);
#endif

  return volkswagen_longitudinal ? BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_LONG_TX_MSGS) : \
                                   BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_STOCK_TX_MSGS);
}

static void volkswagen_meb_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Update vehicle speed and in-motion state by sampling wheel speeds
    // Signals: ESC_51.{VL,VR,HL,HR}_Radgeschw
    if (msg->addr == MSG_ESC_51) {
      uint32_t fl = msg->data[8]  | (msg->data[9]  << 8);
      uint32_t fr = msg->data[10] | (msg->data[11] << 8);
      uint32_t rl = msg->data[12] | (msg->data[13] << 8);
      uint32_t rr = msg->data[14] | (msg->data[15] << 8);
      vehicle_moving = (fl > 0U) || (fr > 0U) || (rl > 0U) || (rr > 0U);
      UPDATE_VEHICLE_SPEED((fl + fr + rl + rr) * (0.0075 / 4.0 / 3.6));
    }

    // Update measured curvature
    // Signal: QFK_01.Curvature, sign in QFK_01.Curvature_VZ
    if (msg->addr == MSG_QFK_01) {
      int curvature = ((msg->data[6] & 0x7FU) << 8) | msg->data[5];
      if (!GET_BIT(msg, 55U)) {
        curvature *= -1;
      }
      update_sample(&angle_meas, curvature);
    }

    // Update driver input torque samples
    if (msg->addr == MSG_LH_EPS_03) {
      update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(msg));
    }

    // Update cruise state and accel pedal
    if (msg->addr == MSG_MOTOR_51) {
      // Signal: Motor_51.TSK_Status
      int acc_status = msg->data[11] & 0x07U;
      bool cruise_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5);
      acc_main_on = cruise_engaged || (acc_status == 2);

      if (!volkswagen_longitudinal) {
        pcm_cruise_check(cruise_engaged);
      }
      if (!acc_main_on) {
        controls_allowed = false;
      }

      // Signal: Motor_51.Accel_Pedal_Pressure
      int accel_pedal = ((msg->data[1] >> 4) & 0x0FU) | ((msg->data[2] & 0x1FU) << 4);
      gas_pressed = accel_pedal > 0;
    }

    // Cruise buttons
    if (msg->addr == MSG_GRA_ACC_01) {
      // If using openpilot longitudinal, enter controls on falling edge of Set or Resume with main switch on
      if (volkswagen_longitudinal) {
        bool set_button = GET_BIT(msg, 16U);
        bool resume_button = GET_BIT(msg, 19U);
        if ((volkswagen_set_button_prev && !set_button) || (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = acc_main_on;
        }
        volkswagen_set_button_prev = set_button;
        volkswagen_resume_button_prev = resume_button;
      }
      // Always exit controls on rising edge of Cancel
      if (GET_BIT(msg, 13U)) {
        controls_allowed = false;
      }
    }

    // Signal: Motor_14.MO_Fahrer_bremst (ECU detected brake pedal switch)
    if (msg->addr == MSG_MOTOR_14) {
      brake_pressed = GET_BIT(msg, 28U);
    }
  }
}

static bool volkswagen_meb_tx_hook(const CANPacket_t *msg) {
  // Curvature is sent on HCA_03 with 6.7e-6 (rad/m)/bit resolution
  static const AngleSteeringLimits VOLKSWAGEN_MEB_STEERING_LIMITS = {
    .max_angle = 29105,                                 // 0.195 rad/m
    .angle_deg_to_can = 149253.7313,                    // 1 / 6.7e-6
    .angle_rate_up_lookup = {{0., 5., 25.}, {0.04, 0.04, 0.005}},
    .angle_rate_down_lookup = {{0., 5., 25.}, {0.04, 0.04, 0.005}},
    .angle_is_curvature = true,
    .inactive_angle_is_zero = true,
  };

  // longitudinal limits
  // acceleration in m/s2 * 1000 to avoid floating point math
  const LongitudinalLimits VOLKSWAGEN_MEB_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 3010,  // VW sends one increment above the max range when inactive
  };

  bool tx = true;

  // Safety check for HCA_03 Heading Control Assist curvature
  if (msg->addr == MSG_HCA_03) {
    int desired_curvature = GET_BYTES(msg, 3, 2) & 0x7FFFU;
    if (!GET_BIT(msg, 39U)) {
      desired_curvature *= -1;
    }
    bool steer_req = ((msg->data[1] >> 4) & 0x0FU) == 4U;
    int steer_power = msg->data[2];

    if (steer_angle_cmd_checks(desired_curvature, steer_req, VOLKSWAGEN_MEB_STEERING_LIMITS)) {
      tx = false;
    }

    // EPS power must be zero unless actively steering, and bounded
    if ((steer_power > 125) || (!steer_req && (steer_power != 0))) {
      tx = false;
    }
  }

  // Safety check for ACC_18 acceleration request
  if (msg->addr == MSG_ACC_18) {
    // Signal: ACC_18.ACC_Sollbeschleunigung_02 (acceleration in m/s2, scale 0.005, offset -7.22)
    int desired_accel = ((((msg->data[4] & 0x7U) << 8) | msg->data[3]) * 5U) - 7220U;
    // MEB driver-override mode: TSK requires accel=0 while gas is pressed
    bool override = controls_allowed && (desired_accel == 0);
    if (!override && longitudinal_accel_checks(desired_accel, VOLKSWAGEN_MEB_LONG_LIMITS)) {
      tx = false;
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((msg->addr == MSG_GRA_ACC_01) && !controls_allowed) {
    if ((msg->data[2] & 0x9U) != 0U) {
      tx = false;
    }
  }

  return tx;
}

const safety_hooks volkswagen_meb_hooks = {
  .init = volkswagen_meb_init,
  .rx = volkswagen_meb_rx_hook,
  .tx = volkswagen_meb_tx_hook,
  .get_counter = volkswagen_mqb_meb_get_counter,
  .get_checksum = volkswagen_mqb_meb_get_checksum,
  .compute_checksum = volkswagen_mqb_meb_compute_crc,
};
