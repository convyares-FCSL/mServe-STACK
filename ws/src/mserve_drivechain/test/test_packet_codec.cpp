#include <gtest/gtest.h>
#include "include/drivechain_uart.hpp"

using mserve_drivechain::DriveUart;

// ==============================================================================
// CRC-8/MAXIM (Dallas 1-Wire) — ground truth values from the ddsm_ctrl reference
// implementation and manually verified against the DDSM115 protocol spec.
// ==============================================================================

TEST(Crc8, ZeroVector)
{
  // CRC of nine 0x00 bytes — known result 0x00 (starting crc=0, polynomial 0x8C).
  uint8_t data[9] = {};
  EXPECT_EQ(DriveUart::crc8(data, 9), 0x00u);
}

TEST(Crc8, IdCheckBroadcastPacket)
{
  // The id-check broadcast packet has a hardcoded CRC of 0xDE.
  // Bytes 0–8: {0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  uint8_t data[9] = {0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(DriveUart::crc8(data, 9), 0xDEu);
}

TEST(Crc8, SpeedCommandPositive)
{
  // Speed command to motor ID=1, rpm=100 (0x0064 in big-endian).
  // Packet bytes 0-8: {0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00}
  uint8_t pkt[10] = {0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  pkt[9] = DriveUart::crc8(pkt, 9);
  // Verify the CRC is stable (same input → same output).
  EXPECT_EQ(DriveUart::crc8(pkt, 9), pkt[9]);
}

TEST(Crc8, SpeedCommandNegative)
{
  // rpm=-100 is int16 = 0xFF9C in big-endian.
  uint8_t pkt[10] = {0x02, 0x64, 0xFF, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t crc = DriveUart::crc8(pkt, 9);
  pkt[9] = crc;
  EXPECT_EQ(DriveUart::crc8(pkt, 9), crc);
  EXPECT_NE(crc, 0x00u);  // sanity: non-trivial packet should not produce 0 CRC
}

TEST(Crc8, ChangeIdPacket)
{
  // change-id header: {0xAA, 0x55, 0x53, new_id, 0x00, 0x00, 0x00, 0x00, 0x00}
  // Test with new_id = 0x01.
  uint8_t pkt[10] = {0xAA, 0x55, 0x53, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t crc = DriveUart::crc8(pkt, 9);
  // CRC must not be 0xDE (that's the broadcast special-case).
  EXPECT_NE(crc, 0xDEu);
  // Round-trip: pkt[9]=crc, crc of full 10 bytes should differ.
  pkt[9] = crc;
  EXPECT_NE(DriveUart::crc8(pkt, 10), crc);  // appending CRC changes result
}

// ==============================================================================
// Sim-mode DriveUart — no hardware required
// ==============================================================================

TEST(DriveUartSim, OpenSucceeds)
{
  DriveUart uart(true);
  EXPECT_TRUE(uart.open("/dev/serial0"));
  EXPECT_TRUE(uart.is_open());
}

TEST(DriveUartSim, SetSpeedReturnsFeedback)
{
  DriveUart uart(true);
  uart.open("/dev/serial0");

  mserve_drivechain::MotorFeedback fb;
  EXPECT_TRUE(uart.set_speed(1, 100, fb));
  EXPECT_EQ(fb.speed_rpm, 100);
  EXPECT_EQ(fb.fault_code, 0);
}

TEST(DriveUartSim, SpeedClamped)
{
  DriveUart uart(true);
  uart.open("/dev/serial0");

  mserve_drivechain::MotorFeedback fb;
  uart.set_speed(1, 500, fb);   // above max
  EXPECT_EQ(fb.speed_rpm, 200);

  uart.set_speed(1, -999, fb);  // below min
  EXPECT_EQ(fb.speed_rpm, -200);
}

TEST(DriveUartSim, PingAlwaysSucceeds)
{
  DriveUart uart(true);
  uart.open("/dev/serial0");
  EXPECT_TRUE(uart.ping(1));
  EXPECT_TRUE(uart.ping(2));
}

TEST(DriveUartSim, IdCheckReturns1)
{
  DriveUart uart(true);
  EXPECT_EQ(uart.id_check(), 1);
}

TEST(DriveUartSim, ChangeIdSucceeds)
{
  DriveUart uart(true);
  uart.open("/dev/serial0");
  EXPECT_TRUE(uart.change_id(1, 5));
}
