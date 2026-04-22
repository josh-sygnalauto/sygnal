// Copyright (c) 2025-present Sygnal Auto, Inc. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "sygnal_can_interface_lib/sygnal_cbfd_interface.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include "sygnal_can_interface_lib/crc8.hpp"

// CBFD protocol requires little-endian float32 packing.
// x86_64 and ARM64 Jetson are both little-endian; std::memcpy of a native float is correct.
// Verify the float size; endianness is guaranteed by the supported platforms.
static_assert(sizeof(float) == 4, "CBFD float32 packing requires sizeof(float) == 4.");

namespace polymath::sygnal
{

std::optional<polymath::socketcan::CanFrame> SygnalCbfdInterface::createCbfdEnableFrame(
  uint8_t device_address,
  uint8_t slot_number,
  bool enable,
  std::string & error_message)
{
  if (slot_number > CBFD_MAX_SLOT) {
    error_message += "CBFD slot_number " + std::to_string(slot_number) +
                     " exceeds maximum (" + std::to_string(CBFD_MAX_SLOT) + ").\n";
    return std::nullopt;
  }

  uint8_t buf[CAN_MAX_DLC] = {
    device_address,
    slot_number,
    static_cast<uint8_t>(enable ? 0x01U : 0x00U),
    0x00, 0x00, 0x00, 0x00,
    0x00,  // CRC placeholder
  };
  buf[7] = generate_crc8(buf);

  std::array<unsigned char, CAN_MAX_DLC> data;
  std::memcpy(data.data(), buf, CAN_MAX_DLC);

  polymath::socketcan::CanFrame frame;
  frame.set_can_id(CBFD_ENABLE_CAN_ID);
  frame.set_len(CAN_MAX_DLC);
  frame.set_data(data);
  return frame;
}

std::optional<polymath::socketcan::CanFrame> SygnalCbfdInterface::createCbfdCommandFrame(
  uint8_t device_address,
  uint8_t slot_number,
  uint8_t counter,
  float value,
  std::string & error_message)
{
  if (slot_number > CBFD_MAX_SLOT) {
    error_message += "CBFD slot_number " + std::to_string(slot_number) +
                     " exceeds maximum (" + std::to_string(CBFD_MAX_SLOT) + ").\n";
    return std::nullopt;
  }

  if (!std::isfinite(value)) {
    error_message += "CBFD command value is not finite.\n";
    return std::nullopt;
  }

  uint8_t buf[CAN_MAX_DLC];
  buf[0] = device_address;
  buf[1] = slot_number;
  buf[2] = counter;
  // Pack float32 as little-endian bytes into buf[3..6].
  std::memcpy(&buf[3], &value, sizeof(float));
  buf[7] = 0x00;           // placeholder before CRC
  buf[7] = generate_crc8(buf);

  std::array<unsigned char, CAN_MAX_DLC> data;
  std::memcpy(data.data(), buf, CAN_MAX_DLC);

  polymath::socketcan::CanFrame frame;
  frame.set_can_id(CBFD_COMMAND_CAN_ID);
  frame.set_len(CAN_MAX_DLC);
  frame.set_data(data);
  return frame;
}

std::optional<polymath::socketcan::CanFrame> SygnalCbfdInterface::createCbfdDisableFrame(
  uint8_t device_address,
  uint8_t slot_number,
  std::string & error_message)
{
  return createCbfdEnableFrame(device_address, slot_number, false, error_message);
}

}  // namespace polymath::sygnal
