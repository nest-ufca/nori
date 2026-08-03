/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2022 Northeastern University
 * Copyright (c) 2022 Sapienza, University of Rome
 * Copyright (c) 2022 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 *
 *
 * Author: Andrea Lacava <thecave003@gmail.com>
 *         Tommaso Zugno <tommasozugno@gmail.com>
 *         Michele Polese <michele.polese@gmail.com>
 */

#pragma once

#include "function-description.h"

#include "ns3/object.h"

#ifndef NORI_ENABLE_RC_V5_CODEC
extern "C"
{
#include "E2SM-RC-RANFunctionDefinition.h"
}
#endif

namespace ns3
{

class RicControlFunctionDescription : public FunctionDescription
{
  public:
    RicControlFunctionDescription();
    ~RicControlFunctionDescription();

  private:
#ifndef NORI_ENABLE_RC_V5_CODEC
    void FillAndEncodeRCFunctionDescription(E2SM_RC_RANFunctionDefinition_t* descriptor);
    void Encode(E2SM_RC_RANFunctionDefinition_t* descriptor);
#endif
};
} // namespace ns3

#pragma once
