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

    command cmd( ds::SET_HKR_CONFIG_TABLE, param1, table_id, param3 );
    for( size_t i = 0; i < n_chunks; ++i )
    {
        auto begin = table.begin() + i * nn::CHUNK_SIZE;
        cmd.data.assign( begin, begin + std::min< size_t >( nn::CHUNK_SIZE, table.end() - begin ) );
        cmd.param4 = nn::chunk_param( n_chunks, i );
        // The base send: chunking is done here, so the extended-buffers override must not chunk again
        hwmon_response_type response;
        _hwm->hw_monitor::send( cmd, &response );
        if( response != ds::d500_hwmon_response::SUCCESS )
            nn::throw_error( response );
        if( progress )
            progress( 0.95f * ( i + 1 ) / n_chunks );
    }

    wait_for_commit();
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
    if( reply.size() < nn::TABLE_HEADER_SIZE + sizeof( status ) )
        throw invalid_value_exception( rsutils::string::from() << "NN status table too short: " << reply.size() << " bytes" );
    std::memcpy( &status, reply.data() + nn::TABLE_HEADER_SIZE, sizeof( status ) );
    return 0;
}


// The last chunk's reply only confirms validation: commit and activation outlast the 1 s XU reply window, so the
// device reports their outcome in its status table
void nn_transport_hwm::wait_for_commit()
{
    auto const deadline = std::chrono::steady_clock::now() + nn::COMMIT_TIMEOUT;
    while( true )
    {
        nn::status status;
        if( auto err = read_status( status ) )
            nn::throw_error( err );
        if( status.state == nn::upload_state::done_reboot_required )
            return;
        if( status.state == nn::upload_state::failed )
            nn::throw_error( status.last_error );
        if( std::chrono::steady_clock::now() > deadline )
            throw io_exception( "NN upload failed: timed out waiting for the device to store the package" );
        std::this_thread::sleep_for( nn::STATUS_POLL_INTERVAL );
    }
}

}  // namespace librealsense
