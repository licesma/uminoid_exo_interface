/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon), Honghyun Kim */

//
// *********         Fast Sync Read Example           *********
//
// Available DYNAMIXEL model on this example : XL330-M288 using Protocol 2.0
// This example is configured for seven XL330-M288 motors, and a U2D2
// Be sure that the motors are set to ID 0 through 6 with baudrate 1000000
//

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses DYNAMIXEL SDK library

// Control table address
constexpr uint16_t ADDR_XL_PRESENT_POSITION = 132;

// Data Byte Length
constexpr uint16_t LEN_XL_PRESENT_POSITION = 4;

// Protocol version
constexpr float PROTOCOL_VERSION = 2.0;

// Default setting
constexpr int BAUDRATE = 1000000;
const char* DEVICENAME = "/dev/ttyUSB1";

const uint8_t DXL_IDS[] = { 1, 2, 3, 4, 5, 6 ,7};
const size_t DXL_ID_COUNT = sizeof(DXL_IDS) / sizeof(DXL_IDS[0]);

int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupFastSyncRead instance for Present Position
  dynamixel::GroupFastSyncRead groupFastSyncRead(portHandler, packetHandler, ADDR_XL_PRESENT_POSITION, LEN_XL_PRESENT_POSITION);

  int dxl_comm_result = COMM_TX_FAIL;               // Communication result
  bool dxl_addparam_result = false;                 // addParam result
  bool dxl_getdata_result = false;                  // GetParam result
  int32_t dxl_present_position = 0;                 // Present position

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    return 1;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    return 1;
  }

  for (size_t i = 0; i < DXL_ID_COUNT; ++i)
  {
    dxl_addparam_result = groupFastSyncRead.addParam(DXL_IDS[i]);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupFastSyncRead addparam failed\n", DXL_IDS[i]);
      return 0;
    }
    printf("DYNAMIXEL#%d is registered for fast sync read\n", DXL_IDS[i]);
  }

  for (int reading = 0; reading < 100; ++reading)
  {
    groupFastSyncRead.txRxPacket();

    printf("[Reading %d]\n", reading + 1);
    for (size_t i = 0; i < DXL_ID_COUNT; ++i)
    {
      dxl_getdata_result = groupFastSyncRead.isAvailable(DXL_IDS[i], ADDR_XL_PRESENT_POSITION, LEN_XL_PRESENT_POSITION);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupFastSyncRead getdata failed\n", DXL_IDS[i]);
        portHandler->closePort();
        return 1;
      }
      dxl_present_position = groupFastSyncRead.getData(DXL_IDS[i], ADDR_XL_PRESENT_POSITION, LEN_XL_PRESENT_POSITION);
      printf("[ID:%03d] Present Position: %d\n", DXL_IDS[i], dxl_present_position);
    }
  }

  // Close port
  portHandler->closePort();

  return 0;
}
