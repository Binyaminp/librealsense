// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

//#cmake:add-file ../../src/nn/nn-hkr-protocol.cpp
//#cmake:add-file ../../src/nn/nn-loader.cpp

#include "../catch.h"
#include <src/nn/nn-hkr-protocol.h>
#include <src/nn/nn-loader.h>
#include <src/librealsense-exception.h>

#include <cstring>
#include <string>

using namespace librealsense;
using Catch::Matchers::ContainsSubstring;


TEST_CASE( "chunk numbering" )
{
    CHECK( nn::chunk_count( 1 ) == 1 );
    CHECK( nn::chunk_count( 1000 ) == 1 );
    CHECK( nn::chunk_count( 1001 ) == 2 );

    CHECK( nn::chunk_param( 1, 0 ) == 0 );
    CHECK( nn::chunk_param( 2, 1 ) == 0x00010001 );
    CHECK( nn::chunk_param( 3, 1 ) == 0x00020001 );

    // The largest file fills exactly 65,536 chunks
    CHECK( nn::chunk_count( nn::MAX_FILE_SIZE + nn::TABLE_HEADER_SIZE ) == nn::MAX_CHUNKS );
    CHECK( nn::chunk_count( nn::MAX_FILE_SIZE + 1 + nn::TABLE_HEADER_SIZE ) == nn::MAX_CHUNKS + 1 );
    CHECK( nn::chunk_param( nn::MAX_CHUNKS, 0 ) == 0xFFFF0000 );
    CHECK( nn::chunk_param( nn::MAX_CHUNKS, nn::MAX_CHUNKS - 1 ) == 0xFFFFFFFF );
}


TEST_CASE( "table header" )
{
    std::string const file = "123456789";
    auto table = nn::build_table( nn::PACKAGE_TABLE_ID, reinterpret_cast< const uint8_t * >( file.data() ), file.size() );
    REQUIRE( table.size() == nn::TABLE_HEADER_SIZE + file.size() );

    auto u16 = [&]( size_t offset ) { uint16_t v; std::memcpy( &v, table.data() + offset, sizeof( v ) ); return v; };
    auto u32 = [&]( size_t offset ) { uint32_t v; std::memcpy( &v, table.data() + offset, sizeof( v ) ); return v; };
    CHECK( u16( 0 ) == nn::TABLE_VERSION );
    CHECK( u16( 2 ) == nn::PACKAGE_TABLE_ID );
    CHECK( u32( 4 ) == file.size() );
    CHECK( u32( 8 ) == 0 );
    CHECK( u32( 12 ) == 0xCBF43926 );  // CRC-32/ISO-HDLC check value
    CHECK( std::string( table.begin() + nn::TABLE_HEADER_SIZE, table.end() ) == file );
}


TEST_CASE( "upload arguments" )
{
    CHECK_NOTHROW( validate_nn_upload_args( 1, RS2_NN_TARGET_FLASH, 0, true ) );
    CHECK_NOTHROW( validate_nn_upload_args( nn::MAX_FILE_SIZE, RS2_NN_TARGET_FLASH, 0, true ) );

    CHECK_THROWS_AS( validate_nn_upload_args( 0, RS2_NN_TARGET_FLASH, 0, true ), invalid_value_exception );
    CHECK_THROWS_AS( validate_nn_upload_args( nn::MAX_FILE_SIZE + 1, RS2_NN_TARGET_FLASH, 0, true ), invalid_value_exception );
    CHECK_THROWS_AS( validate_nn_upload_args( 1, RS2_NN_TARGET_DDR, 0, true ), not_implemented_exception );
    CHECK_THROWS_AS( validate_nn_upload_args( 1, RS2_NN_TARGET_FLASH, 1, true ), not_implemented_exception );
    CHECK_THROWS_AS( validate_nn_upload_args( 1, RS2_NN_TARGET_FLASH, 0, false ), not_implemented_exception );
}


static std::string thrown_message( int32_t code )
{
    try
    {
        nn::throw_error( code );
    }
    catch( librealsense_exception const & e )
    {
        return e.what();
    }
    return {};
}


TEST_CASE( "error mapping" )
{
    CHECK_THROWS_AS( nn::throw_error( -4 ), not_implemented_exception );
    CHECK_THAT( thrown_message( -4 ), ContainsSubstring( "locked" ) );
    CHECK_THROWS_AS( nn::throw_error( -12 ), not_implemented_exception );
    CHECK_THROWS_AS( nn::throw_error( -10 ), invalid_value_exception );
    CHECK_THAT( thrown_message( -10 ), ContainsSubstring( "too large" ) && ContainsSubstring( "(-10)" ) );
    CHECK( nn::error_message( -99 ) == "device error -99" );
}
