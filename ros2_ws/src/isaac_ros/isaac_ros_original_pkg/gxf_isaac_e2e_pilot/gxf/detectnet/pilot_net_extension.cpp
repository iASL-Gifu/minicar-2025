// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#include "gxf/pilot_net/ackermann_message.hpp"
#include "gxf/pilot_net/pilot_net_decoder.hpp"
#include "gxf/std/extension_factory_helper.hpp"

GXF_EXT_FACTORY_BEGIN()

GXF_EXT_FACTORY_SET_INFO(
  0x1a885739160245e2, 0x9ef19134f30ad92f, "PilotNetExtension",
  "GXF extension for E2E PilotNet components",
  "NVIDIA", "1.0.0", "LICENSE");

// デコーダーCodeletを登録
GXF_EXT_FACTORY_ADD(
  0x8bbd40aa9bb340dd, 0x84e81a21e0603442,
  nvidia::isaac_ros::PilotNetDecoder, nvidia::gxf::Codelet,
  "Decodes PilotNet output tensor to AckermannDrive message.");

// カスタムメッセージ型を登録
GXF_EXT_FACTORY_ADD_0(
  0x9fc9101525594104, 0xbf12d9f22a134906,
  nvidia::isaac_ros::AckermannDrive,
  "Custom message component for AckermannDrive data.");

GXF_EXT_FACTORY_END()