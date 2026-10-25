// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "nn-transport.h"

#include <src/librealsense-exception.h>

#include <rsutils/string/from.h>

#include <algorithm>
#include <thread>

namespace librealsense {


void nn_transport::send_table( uint16_t table_id, uint32_t param1, uint32_t param3,
                               const std::vector< uint8_t > & table, const progress_callback & progress )
{
    size_t const n_chunks = nn::chunk_count( table.size() );
    if( ! n_chunks || n_chunks > nn::MAX_CHUNKS )
        throw invalid_value_exception( rsutils::string::from() << "invalid HKR table size " << table.size() );

    auto const before = get_status();
    if( nn::is_busy( before.state ) )
        throw io_exception( std::string( "NN upload failed: " ) + nn::BUSY_MESSAGE );

    for( size_t i = 0; i < n_chunks; ++i )
    {
        size_t const offset = i * nn::CHUNK_SIZE;
        if( auto err = send_chunk( table_id, param1, param3, nn::chunk_param( n_chunks, i ), table.data() + offset,
                                   std::min( nn::CHUNK_SIZE, table.size() - offset ) ) )
            nn::throw_error( err );
        if( progress )
            progress( 0.9f * ( i + 1 ) / n_chunks );
    }

    wait_for_commit( before.upload_seq + 1, progress );
    if( progress )
        progress( 1.f );
}


nn::status nn_transport::get_status()
{
    nn::status status;
    if( auto err = read_status( status ) )
        nn::throw_error( err );
    return status;
}


// The last chunk's reply comes before the commit, which outlasts the 1 s XU reply window: the device reports its
// progress and result in NN_STATUS, under the sequence number of this upload
void nn_transport::wait_for_commit( uint32_t upload_seq, const progress_callback & progress )
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
        if( now - last_change > _timing.stall_timeout || now - start > _timing.commit_timeout )
            throw io_exception( "NN upload failed: timed out waiting for the device to store the package" );
        last = status;
        std::this_thread::sleep_for( _timing.poll_interval );
    }
}

}  // namespace librealsense
