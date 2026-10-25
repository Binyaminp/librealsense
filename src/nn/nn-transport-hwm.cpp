// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "nn-transport-hwm.h"

#include <src/uvc-sensor.h>
#include <src/ds/d500/d500-private.h>

#include <rsutils/string/from.h>

#include <algorithm>
#include <cstring>
#include <thread>

namespace librealsense {

static_assert( nn::CHUNK_SIZE == HW_MONITOR_COMMAND_SIZE, "HKR table chunks must fill an HW-monitor command" );


nn_transport_hwm::nn_transport_hwm( std::shared_ptr< hw_monitor > hwm, std::shared_ptr< uvc_sensor > raw_depth )
    : _hwm( std::move( hwm ) )
    , _raw_depth( std::move( raw_depth ) )
{
}


void nn_transport_hwm::send_table( uint16_t table_id, uint32_t param1, uint32_t param3,
                                   const std::vector< uint8_t > & table, const progress_callback & progress )
{
    size_t const n_chunks = nn::chunk_count( table.size() );
    if( ! n_chunks || n_chunks > nn::MAX_CHUNKS )
        throw invalid_value_exception( rsutils::string::from() << "invalid HKR table size " << table.size() );

    auto power = _raw_depth->bulk_operation();
    auto const before = get_status();
    if( nn::is_busy( before.state ) )
        throw io_exception( "NN upload failed: another upload is in progress on the device" );

    command cmd( ds::SET_HKR_CONFIG_TABLE, param1, table_id, param3 );
    for( size_t i = 0; i < n_chunks; ++i )
    {
        auto begin = table.begin() + i * nn::CHUNK_SIZE;
        cmd.data.assign( begin, begin + std::min( nn::CHUNK_SIZE, static_cast< size_t >( table.end() - begin ) ) );
        cmd.param4 = nn::chunk_param( n_chunks, i );
        // The base send: chunking is done here, so the extended-buffers override must not chunk again
        hwmon_response_type response;
        _hwm->hw_monitor::send( cmd, &response );
        if( response != ds::d500_hwmon_response::SUCCESS )
            nn::throw_error( response );
        if( progress )
            progress( 0.9f * ( i + 1 ) / n_chunks );
    }

    wait_for_commit( before.upload_seq + 1, progress );
    if( progress )
        progress( 1.f );
}


int32_t nn_transport_hwm::read_status( nn::status & status )
{
    command cmd( ds::GET_HKR_CONFIG_TABLE, nn::STATUS_REGION, nn::STATUS_TABLE_ID );
    hwmon_response_type response;
    auto reply = _hwm->hw_monitor::send( cmd, &response );
    if( response != ds::d500_hwmon_response::SUCCESS )
        return response;
    if( reply.size() < sizeof( status ) )
        throw invalid_value_exception( rsutils::string::from() << "NN status table too short: " << reply.size() << " bytes" );
    std::memcpy( &status, reply.data(), sizeof( status ) );
    return 0;
}


nn::status nn_transport_hwm::get_status()
{
    nn::status status;
    if( auto err = read_status( status ) )
        nn::throw_error( err );
    return status;
}


// The last chunk's reply comes before the commit, which outlasts the 1 s XU reply window: the device reports its
// progress and result in NN_STATUS, under the sequence number of this upload
void nn_transport_hwm::wait_for_commit( uint32_t upload_seq, const progress_callback & progress )
{
    auto const start = std::chrono::steady_clock::now();
    auto last_change = start;
    nn::status last = {};
    while( true )
    {
        auto status = get_status();
        auto const now = std::chrono::steady_clock::now();
        if( status.upload_seq == upload_seq )
        {
            if( status.state == nn::upload_state::done )
                return;
            if( status.state == nn::upload_state::error )
                nn::throw_error( status.error );
            if( progress )
                progress( 0.9f + 0.001f * std::min< uint8_t >( status.progress, 100 ) );
        }
        if( status.upload_seq != last.upload_seq || status.state != last.state || status.progress != last.progress )
            last_change = now;
        if( now - last_change > nn::STALL_TIMEOUT || now - start > nn::COMMIT_TIMEOUT )
            throw io_exception( "NN upload failed: timed out waiting for the device to store the package" );
        last = status;
        std::this_thread::sleep_for( nn::STATUS_POLL_INTERVAL );
    }
}

}  // namespace librealsense
