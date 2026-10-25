// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#pragma once

#include "nn-hkr-protocol.h"

#include <functional>

namespace librealsense {

// Moves an HKR table to the device. Implemented per link, so other channels (e.g. a bulk endpoint) can be added later.
class nn_transport
{
public:
    using progress_callback = std::function< void( float ) >;

    virtual ~nn_transport() = default;

    // Sends the whole table (header included) and returns once the device has committed it; throws on any device error
    virtual void send_table( uint16_t table_id, uint32_t param1, uint32_t param3,
                             const std::vector< uint8_t > & table, const progress_callback & progress ) = 0;

    // Returns 0 and fills the status, or the device error code
    virtual int32_t read_status( nn::status & status ) = 0;
};

}  // namespace librealsense
