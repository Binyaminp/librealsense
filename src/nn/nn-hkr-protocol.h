// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#pragma once

#include <librealsense2/h/rs_nn.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace librealsense {
namespace nn {

// Host-to-HKR NN package protocol over SET/GET_HKR_CONFIG_TABLE (0xA6/0xA7).
// Tables come from the virtual-table range (0x0A00-0x0AFF). The device replies to the last chunk once the package is
// received, then validates, writes and activates it in the background; progress and result are read from NN_STATUS.
// Values below follow the draft HKR FW interface spec and are expected to change until it is frozen.
constexpr uint16_t PACKAGE_TABLE_ID = 0x0A04;  // NN_MODEL
constexpr uint16_t STATUS_TABLE_ID = 0x0A07;   // NN_STATUS
constexpr uint16_t TABLE_VERSION = 0x0100;
constexpr uint32_t STATUS_REGION = 2;  // RAM
constexpr std::chrono::milliseconds STATUS_POLL_INTERVAL( 100 );
constexpr std::chrono::seconds STALL_TIMEOUT( 10 );  // no change in state or progress
constexpr std::chrono::seconds COMMIT_TIMEOUT( 60 );

// param4 carries the chunk index in 16 bits, and HKR tables carry a 16-byte header in front of the payload
constexpr size_t CHUNK_SIZE = 1000;
constexpr size_t MAX_CHUNKS = 1 << 16;
constexpr size_t TABLE_HEADER_SIZE = 16;
constexpr size_t MAX_FILE_SIZE = MAX_CHUNKS * CHUNK_SIZE - TABLE_HEADER_SIZE;

enum class upload_state : uint8_t
{
    idle,
    receiving,
    validating,
    decompressing,
    writing_flash,
    activating,
    done,
    error,
};

constexpr const char * BUSY_MESSAGE = "another upload is in progress on the device";
inline bool is_busy( upload_state state ) { return state >= upload_state::receiving && state <= upload_state::activating; }

// NN_STATUS, little-endian, after the HKR table header; readers ignore bytes beyond the fields they know
#pragma pack( push, 1 )
struct status
{
    uint16_t version;
    uint16_t size;
    upload_state state;
    uint8_t slot;
    uint8_t progress;  // 0-100% of the commit
    uint8_t reserved;
    int32_t error;
    uint32_t upload_seq;  // incremented by the device on every chunk 0, if it fills it
    uint32_t bytes_received;
    uint32_t total_bytes;
    uint32_t elapsed_ms;
};
#pragma pack( pop )
static_assert( sizeof( status ) == 28, "unexpected NN_STATUS size" );

// The HKR table header (with CRC-32 of the file) followed by the file, unchanged
std::vector< uint8_t > build_table( uint16_t table_id, const uint8_t * file, size_t size );

// The status from an NN_STATUS reply: header and CRC are checked
status parse_status( const std::vector< uint8_t > & reply );

// param1 of a table write: 1 = flash, 2 = DDR
inline uint32_t target_param( rs2_nn_target target ) { return target == RS2_NN_TARGET_DDR ? 2 : 1; }

inline size_t chunk_count( size_t table_size ) { return ( table_size + CHUNK_SIZE - 1 ) / CHUNK_SIZE; }

// param4 of chunk i out of n: ( n - 1 ) << 16 | i
inline uint32_t chunk_param( size_t n_chunks, size_t i ) { return static_cast< uint32_t >( ( n_chunks - 1 ) << 16 | i ); }

std::string error_message( int32_t code );

// Throws the library exception matching an HKR error code
[[noreturn]] void throw_error( int32_t code );

}  // namespace nn
}  // namespace librealsense
