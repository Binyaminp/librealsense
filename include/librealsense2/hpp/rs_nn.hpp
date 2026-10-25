// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#ifndef LIBREALSENSE_RS2_NN_HPP
#define LIBREALSENSE_RS2_NN_HPP

#include "rs_types.hpp"
#include "rs_device.hpp"
#include "../h/rs_nn.h"

namespace rs2
{
    /**
     * \brief Device that accepts NN model packages (.hkrp files from the Perception Service)
     */
    class nn_loader : public device
    {
    public:
        nn_loader() : device() {}

        nn_loader( device d ) : device( d.get() )
        {
            rs2_error * e = nullptr;
            if( ! rs2_is_device_extendable_to( _dev.get(), RS2_EXTENSION_NN_LOADER, &e ) || e )
                _dev.reset();
            error::handle( e );
        }

        /**
         * Upload an NN model package, sent to the device as is. Stop the Perception sensor first, and reboot the
         * device (hardware_reset) afterwards for the new model to take effect.
         * Currently only RS2_NN_TARGET_FLASH, slot 0 and activate = true are supported.
         */
        void nn_upload( const std::vector< uint8_t > & file,
                        rs2_nn_target target = RS2_NN_TARGET_FLASH,
                        int slot = 0,
                        bool activate = true ) const
        {
            rs2_error * e = nullptr;
            rs2_nn_upload_cpp( _dev.get(), file.data(), int( file.size() ), target, slot, activate, nullptr, &e );
            error::handle( e );
        }

        /**
         * Same as above, with progress notifications in [0, 1] via the callback
         */
        template< class T >
        void nn_upload( const std::vector< uint8_t > & file, rs2_nn_target target, int slot, bool activate,
                        T callback ) const
        {
            rs2_error * e = nullptr;
            rs2_nn_upload_cpp( _dev.get(), file.data(), int( file.size() ), target, slot, activate,
                               new update_progress_callback< T >( std::move( callback ) ), &e );
            error::handle( e );
        }

        /**
         * Whether the device can accept an upload now (e.g. firmware support, not locked)
         * \param[out] reason  If non-null, receives why not
         */
        bool is_upload_supported( std::string * reason = nullptr ) const
        {
            rs2_error * e = nullptr;
            char buf[256] = {};
            bool supported = rs2_nn_is_upload_supported( _dev.get(), buf, sizeof( buf ), &e ) != 0;
            error::handle( e );
            if( reason )
                *reason = buf;
            return supported;
        }
    };
}
#endif // LIBREALSENSE_RS2_NN_HPP
