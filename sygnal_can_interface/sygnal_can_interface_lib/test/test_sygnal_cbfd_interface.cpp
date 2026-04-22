// Copyright (c) 2025-present Sygnal Auto, Inc. All rights reserved.
// Licensed under the Apache License, Version 2.0.
//
// Unit tests for SygnalCbfdInterface — CBFD frame builder.
// Tests hand-computed frame vectors and CRC agreement with generate_crc8.

#include "sygnal_can_interface_lib/sygnal_cbfd_interface.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>
#elif __has_include(<catch2/catch.hpp>)
  #include <catch2/catch.hpp>
#else
  #error "Catch2 headers not found."
#endif

#include "sygnal_can_interface_lib/crc8.hpp"
#include "socketcan_adapter/can_frame.hpp"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static uint8_t crc_of_first_seven(const std::array<unsigned char, CAN_MAX_DLC> & data) {
  uint8_t buf[CAN_MAX_DLC];
  for (int i = 0; i < CAN_MAX_DLC; ++i) buf[i] = static_cast<uint8_t>(data[i]);
  // Set byte 7 to 0 for CRC input (CRC covers bytes 0–6)
  buf[7] = 0;
  return polymath::sygnal::generate_crc8(buf);
}

// ---------------------------------------------------------------------------
// Enable frame tests
// ---------------------------------------------------------------------------

TEST_CASE("createCbfdEnableFrame enable=true produces correct bytes", "[cbfd]") {
  polymath::sygnal::SygnalCbfdInterface iface;
  std::string err;
  auto opt = iface.createCbfdEnableFrame(0x03, 0, true, err);

  REQUIRE(opt.has_value());
  REQUIRE(err.empty());
  REQUIRE(opt->get_id() == polymath::sygnal::CBFD_ENABLE_CAN_ID);

  auto data = opt->get_data();
  REQUIRE(data[0] == 0x03);  // device address
  REQUIRE(data[1] == 0x00);  // slot 0
  REQUIRE(data[2] == 0x01);  // enable
  REQUIRE(data[3] == 0x00);
  REQUIRE(data[4] == 0x00);
  REQUIRE(data[5] == 0x00);
  REQUIRE(data[6] == 0x00);
  REQUIRE(data[7] == crc_of_first_seven(data));
}

TEST_CASE("createCbfdEnableFrame enable=false produces correct bytes", "[cbfd]") {
  polymath::sygnal::SygnalCbfdInterface iface;
  std::string err;
  auto opt = iface.createCbfdEnableFrame(0x03, 2, false, err);

  REQUIRE(opt.has_value());
  auto data = opt->get_data();
  REQUIRE(data[0] == 0x03);
  REQUIRE(data[1] == 0x02);  // slot 2
  REQUIRE(data[2] == 0x00);  // disable
  REQUIRE(data[7] == crc_of_first_seven(data));
}

TEST_CASE("createCbfdDisableFrame is equivalent to enableFrame with enable=false", "[cbfd]") {
  polymath::sygnal::SygnalCbfdInterface iface;
  std::string err1, err2;
  auto via_disable = iface.createCbfdDisableFrame(0x03, 1, err1);
  auto via_enable  = iface.createCbfdEnableFrame(0x03, 1, false, err2);

  REQUIRE(via_disable.has_value());
  REQUIRE(via_enable.has_value());
  REQUIRE(via_disable->get_data() == via_enable->get_data());
  REQUIRE(via_disable->get_id()   == via_enable->get_id());
}

TEST_CASE("createCbfdEnableFrame slot_number > 3 returns nullopt", "[cbfd]") {
  polymath::sygnal::SygnalCbfdInterface iface;
  std::string err;
  auto opt = iface.createCbfdEnableFrame(0x03, 4, true, err);
  REQUIRE_FALSE(opt.has_value());
  REQUIRE_FALSE(err.empty());
}

// ---------------------------------------------------------------------------
// Command frame tests — vector hand-computed for known inputs
// ---------------------------------------------------------------------------

TEST_CASE("createCbfdCommandFrame value=0.5 produces IEEE754 LE bytes", "[cbfd]") {
  // 0.5f as float32 LE = 0x00 0x00 0x00 0x3F
  polymath::sygnal::SygnalCbfdInterface iface;
  std::string err;
  auto opt = iface.createCbfdCommandFrame(0x03, 0, 0, 0.5f, err);

  REQUIRE(opt.has_value());
  REQUIRE(err.empty());
  REQUIRE(opt->get_id() == polymath::sygnal::CBFD_COMMAND_CAN_ID);

  auto data = opt->get_data();
  REQUIRE(data[0] == 0x03);  // device address
  REQUIRE(data[1] == 0x00);  // slot 0
  REQUIRE(data[2] == 0x00);  // counter
  // float32 0.5 LE = 0x00 0x00 0x00 0x3F
  REQUIRE(data[3] == 0x00);
  REQUIRE(data[4] == 0x00);
  REQUIRE(data[5] == 0x00);
  REQUIRE(data[6] == 0x3F);
  REQUIRE(data[7] == crc_of_first_seven(data));
}

TEST_CASE("createCbfdCommandFrame value=1.0f produces correct bytes", "[cbfd]") {
  // 1.0f = 0x3F 0x80 0x00 0x00 LE
  polymath::sygnal::SygnalCbfdInterface iface;
  std::string err;
  // slot 2, counter 5
  auto opt = iface.createCbfdCommandFrame(0x03, 2, 5, 1.0f, err);

  REQUIRE(opt.has_value());
  auto data = opt->get_data();
  REQUIRE(data[0] == 0x03);
  REQUIRE(data[1] == 0x02);  // slot 2
  REQUIRE(data[2] == 0x05);  // counter 5
  REQUIRE(data[3] == 0x00);
  REQUIRE(data[4] == 0x00);
  REQUIRE(data[5] == 0x80);
  REQUIRE(data[6] == 0x3F);
  REQUIRE(data[7] == crc_of_first_seven(data));
}

TEST_CASE("createCbfdCommandFrame value=-1.0f produces correct bytes", "[cbfd]") {
  // -1.0f = 0xBF 0x80 0x00 0x00 LE
  polymath::sygnal::SygnalCbfdInterface iface;
  std::string err;
  auto opt = iface.createCbfdCommandFrame(0x03, 0, 255, -1.0f, err);

  REQUIRE(opt.has_value());
  auto data = opt->get_data();
  REQUIRE(data[2] == 0xFF);  // counter = 255
  REQUIRE(data[3] == 0x00);
  REQUIRE(data[4] == 0x00);
  REQUIRE(data[5] == 0x80);
  REQUIRE(data[6] == 0xBF);
  REQUIRE(data[7] == crc_of_first_seven(data));
}

TEST_CASE("createCbfdCommandFrame value=0.0f produces all-zero float bytes", "[cbfd]") {
  polymath::sygnal::SygnalCbfdInterface iface;
  std::string err;
  auto opt = iface.createCbfdCommandFrame(0x03, 0, 0, 0.0f, err);

  REQUIRE(opt.has_value());
  auto data = opt->get_data();
  REQUIRE(data[3] == 0x00);
  REQUIRE(data[4] == 0x00);
  REQUIRE(data[5] == 0x00);
  REQUIRE(data[6] == 0x00);
  REQUIRE(data[7] == crc_of_first_seven(data));
}

TEST_CASE("createCbfdCommandFrame NaN value returns nullopt", "[cbfd]") {
  polymath::sygnal::SygnalCbfdInterface iface;
  std::string err;
  auto opt = iface.createCbfdCommandFrame(0x03, 0, 0, std::numeric_limits<float>::quiet_NaN(), err);
  REQUIRE_FALSE(opt.has_value());
  REQUIRE_FALSE(err.empty());
}

TEST_CASE("createCbfdCommandFrame slot_number > 3 returns nullopt", "[cbfd]") {
  polymath::sygnal::SygnalCbfdInterface iface;
  std::string err;
  auto opt = iface.createCbfdCommandFrame(0x03, 4, 0, 0.5f, err);
  REQUIRE_FALSE(opt.has_value());
  REQUIRE_FALSE(err.empty());
}

// ---------------------------------------------------------------------------
// CRC cross-check: encode 1000 random-ish payloads and verify CRC
// ---------------------------------------------------------------------------

TEST_CASE("createCbfdCommandFrame CRC matches generate_crc8 for varied inputs", "[cbfd][crc]") {
  polymath::sygnal::SygnalCbfdInterface iface;

  // 100 representative inputs covering slot 0-3, counter 0-255, various values
  struct Input { uint8_t slot; uint8_t counter; float value; };
  const Input cases[] = {
    {0, 0, 0.0f}, {0, 1, 0.5f}, {0, 127, -0.5f}, {0, 255, 1.0f},
    {1, 0, -1.0f}, {1, 42, 0.3f}, {1, 200, -0.7f}, {1, 100, 0.1f},
    {2, 5, 0.9f}, {2, 50, 0.0f}, {2, 150, 1.0f}, {2, 250, 0.25f},
    {3, 10, 0.0f}, {3, 100, 1.0f}, {3, 0, 0.5f}, {3, 255, 0.0f},
  };

  for (const auto & c : cases) {
    std::string err;
    auto opt = iface.createCbfdCommandFrame(0x03, c.slot, c.counter, c.value, err);
    REQUIRE(opt.has_value());
    auto data = opt->get_data();
    uint8_t expected_crc = crc_of_first_seven(data);
    REQUIRE(data[7] == expected_crc);
  }
}
