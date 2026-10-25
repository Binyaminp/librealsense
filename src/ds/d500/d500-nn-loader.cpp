// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "d500-nn-loader.h"
#include "d500-info.h"
#include "d500-private.h"

#include <src/perception-sensor.h>
#include <src/uvc-sensor.h>
#include <src/nn/nn-transport-hwm.h>

namespace librealsense
{
    // First FW with the NN package tables; a placeholder until FW releases the feature
    static const firmware_version NN_UPLOAD_MIN_FW( "7.60.0.0" );

    d500_nn_loader::d500_nn_loader( std::shared_ptr< const d500_info > const & dev_info )
        : device( dev_info )
        , d500_device( dev_info )
    {
    }

    bool d500_nn_loader::supports_nn_upload() const
    {
        return ! _is_mipi_device && _fw_version >= NN_UPLOAD_MIN_FW;
    }

    // The lock state is read fresh, as it can change while the device is connected
    bool d500_nn_loader::is_locked_now() const
    {
        std::vector< uint8_t > gvd( HW_MONITOR_BUFFER_SIZE );
        _hw_monitor->get_gvd( gvd.size(), gvd.data(), ds::fw_cmd::GVD );
        return gvd[ds::d500_gvd_offsets::is_camera_locked_offset] != 0;
    }

    void d500_nn_loader::nn_upload( const uint8_t * file, size_t size, rs2_nn_target target, int slot, bool activate,
                                    rs2_update_progress_callback_sptr callback )
    {
        validate_nn_upload_args( size, target, slot, activate );
        if( ! supports_nn_upload() )
            throw not_implemented_exception( nn::error_message( ds::d500_hwmon_response::PARAMS_TABLE_ID_NOT_VALID ) );

        std::lock_guard< std::mutex > lock( _nn_upload_mutex );
        for( size_t i = 0; i < get_sensors_count(); ++i )
            if( dynamic_cast< perception_sensor * >( &get_sensor( i ) ) && get_sensor( i ).is_streaming() )
                throw wrong_api_call_sequence_exception( "NN upload: stop the Perception sensor before uploading" );
        if( is_locked_now() )
            nn::throw_error( ds::d500_hwmon_response::UNAUTHORIZED_USER_ACTION );

        // Slot and activate go into param3 once FW defines its layout; until then validation admits only slot 0 + activate
        auto power = get_raw_depth_sensor()->bulk_operation();
        nn_transport_hwm( _hw_monitor )
            .send_table( nn::PACKAGE_TABLE_ID, nn::target_param( target ), 0, nn::build_table( nn::PACKAGE_TABLE_ID, file, size ),
                         [&]( float progress ) { if( callback ) callback->on_update_progress( progress ); } );
    }

    bool d500_nn_loader::is_upload_supported( std::string & reason )
    {
        int32_t err = ds::d500_hwmon_response::SUCCESS;
        nn::status status;
        if( ! supports_nn_upload() )
            err = ds::d500_hwmon_response::PARAMS_TABLE_ID_NOT_VALID;
        else if( is_locked_now() )
            err = ds::d500_hwmon_response::UNAUTHORIZED_USER_ACTION;
        else
            err = nn_transport_hwm( _hw_monitor ).read_status( status );
        if( err != ds::d500_hwmon_response::SUCCESS )
            reason = nn::error_message( err );
        return err == ds::d500_hwmon_response::SUCCESS;
    }
}
