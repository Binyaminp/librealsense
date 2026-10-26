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

    wait_for_commit( before.upload_seq, progress );
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
// progress and result in NN_STATUS. It resets the state on chunk 0 and, if it fills upload_seq, increments it, so a
// status still carrying the sequence number seen before the upload is this upload's too.
// Only a total limit: progress stays 0 for the whole flash write. If the status can't be read, the result is unknown.
void nn_transport::wait_for_commit( uint32_t seq_before, const progress_callback & progress )
{
    auto const start = std::chrono::steady_clock::now();
    while( true )
    {
        nn::status status;
        std::string failure;
        try
        {
            if( auto err = read_status( status ) )
                failure = nn::error_message( err );
        }
        catch( std::exception const & e )
        {
            failure = e.what();
        }
        if( ! failure.empty() )
            throw io_exception( rsutils::string::from() << nn::INCOMPLETE_MESSAGE << " (" << failure << ")" );
        if( status.upload_seq == seq_before || status.upload_seq == seq_before + 1 )
        {
            if( status.state == nn::upload_state::done )
                return;
            if( status.state == nn::upload_state::error )
                nn::throw_error( status.error );
            if( progress )
                progress( 0.9f + 0.001f * std::min< uint8_t >( status.progress, 100 ) );
        }
        if( std::chrono::steady_clock::now() - start > _timing.commit_timeout )
            throw io_exception( nn::INCOMPLETE_MESSAGE );
        std::this_thread::sleep_for( _timing.poll_interval );
    }
}

}  // namespace librealsense
