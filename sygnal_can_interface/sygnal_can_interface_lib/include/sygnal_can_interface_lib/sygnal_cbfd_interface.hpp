// Copyright (c) 2025-present Sygnal Auto, Inc. All rights reserved.
// Licensed under the Apache License, Version 2.0.
//
// CBFD (CAN Bridge Full-Duplex) frame builder.
// Peer to SygnalControlInterface for the CBFD secondary-control device.
//
// Protocol (user-supplied spec):
//   Enable slot: CAN ID 0x060, bytes {addr, slot, 0x01|0x00, 0,0,0,0, CRC8}
//   Command slot: CAN ID 0x160, bytes {addr, slot, counter, float32_LE[0:4], CRC8}
//
// CRC-8/SMBUS: polynomial 0x07, init 0x00 — same as generate_crc8() in crc8.hpp.

#ifndef SYGNAL_CAN_INTERFACE_LIB__SYGNAL_CBFD_INTERFACE_HPP_
#define SYGNAL_CAN_INTERFACE_LIB__SYGNAL_CBFD_INTERFACE_HPP_

#include <optional>
#include <string>

#include "socketcan_adapter/can_frame.hpp"

namespace polymath::sygnal
{

constexpr uint32_t CBFD_ENABLE_CAN_ID  = 0x060;
constexpr uint32_t CBFD_COMMAND_CAN_ID = 0x160;

constexpr uint8_t CBFD_MAX_SLOT = 3;

class SygnalCbfdInterface
{
public:
  SygnalCbfdInterface() = default;
  ~SygnalCbfdInterface() = default;

  /// @brief Build a CBFD enable or disable frame (CAN ID 0x060).
  /// @param device_address CBFD device address (typically 3).
  /// @param slot_number Slot to enable/disable (0–3).
  /// @param enable true = enable slot, false = disable slot.
  /// @param error_message Populated on validation failure.
  /// @return Populated CanFrame, or std::nullopt on error.
  std::optional<polymath::socketcan::CanFrame> createCbfdEnableFrame(
    uint8_t device_address,
    uint8_t slot_number,
    bool enable,
    std::string & error_message);

  /// @brief Build a CBFD command frame (CAN ID 0x160).
  /// @param device_address CBFD device address (typically 3).
  /// @param slot_number Slot to command (0–3).
  /// @param counter Rolling 8-bit counter (incremented per send; wraps 255→0).
  /// @param value Float32 value — caller must pre-clamp to slot's [min, max].
  ///              NaN or Inf → error.
  /// @param error_message Populated on validation failure.
  /// @return Populated CanFrame, or std::nullopt on error.
  std::optional<polymath::socketcan::CanFrame> createCbfdCommandFrame(
    uint8_t device_address,
    uint8_t slot_number,
    uint8_t counter,
    float value,
    std::string & error_message);

  /// @brief Convenience: createCbfdEnableFrame(addr, slot, false, err).
  std::optional<polymath::socketcan::CanFrame> createCbfdDisableFrame(
    uint8_t device_address,
    uint8_t slot_number,
    std::string & error_message);
};

}  // namespace polymath::sygnal

#endif  // SYGNAL_CAN_INTERFACE_LIB__SYGNAL_CBFD_INTERFACE_HPP_
