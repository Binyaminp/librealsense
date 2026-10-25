// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "nn-hkr-protocol.h"

#include <src/librealsense-exception.h>
#include <src/ds/d500/d500-types/common.h>

#include <rsutils/number/crc32.h>
#include <rsutils/string/from.h>

#include <cstring>

namespace librealsense {
namespace nn {

static_assert( sizeof( table_header ) == TABLE_HEADER_SIZE, "unexpected HKR table header size" );


std::vector< uint8_t > build_table( uint16_t table_id, const uint8_t * file, size_t size )
{
    table_header header( TABLE_VERSION, table_id, static_cast< uint32_t >( size ), 0, rsutils::number::calc_crc32( file, size ) );
    std::vector< uint8_t > table( sizeof( header ) + size );
    std::memcpy( table.data(), &header, sizeof( header ) );
    std::memcpy( table.data() + sizeof( header ), file, size );
    return table;
}


std::string error_message( int32_t code )
{
    switch( code )
    {
    case -1: return "the NN tables are not supported by this firmware";
    case -2: return "invalid upload parameter";
    case -3: return "a model in the device file system overrides the flash slot; remove it before uploading";
    case -4: return "NN upload is not supported on locked cameras";
    case -5: return "invalid package, or the NN metadata does not match the weights";
    case -6: return "package CRC mismatch";
    case -10: return "the model metadata or weights are too large for their flash slot";
    case -12: return "NN upload is not supported by this firmware";
    case -15: return "flash write failed";
    case -19: return "upload target is not supported by this firmware";
    case -20: return "incomplete transfer";
    case -21: return "device is not ready";
    case -41: return "device command timed out";
    default: return rsutils::string::from() << "device error " << code;
    }
}


void throw_error( int32_t code )
{
    std::string msg = rsutils::string::from() << "NN upload failed: " << error_message( code ) << " (" << code << ")";
    if( code == -1 || code == -4 || code == -12 || code == -19 )
        throw not_implemented_exception( msg );
    throw invalid_value_exception( msg );
}

}  // namespace nn
}  // namespace librealsense
