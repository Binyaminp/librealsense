/* License: Apache 2.0. See LICENSE file in root directory.
Copyright(c) 2026 RealSense, Inc. All Rights Reserved. */

#include "pyrealsense2.h"
#include <librealsense2/hpp/rs_nn.hpp>

#include <fstream>
#include <iterator>

static void nn_upload( const rs2::nn_loader & self, const std::vector< uint8_t > & file, rs2_nn_target target, int slot,
                       bool activate, std::function< void( float ) > callback )
{
    if( callback )
        self.nn_upload( file, target, slot, activate, std::move( callback ) );
    else
        self.nn_upload( file, target, slot, activate );
}

void init_nn( py::module & m )
{
    /** rs_nn.hpp **/
    BIND_ENUM( m, rs2_nn_target, RS2_NN_TARGET_COUNT, "Where an uploaded NN model is stored." )

    py::class_< rs2::nn_loader, rs2::device, py_holder< rs2::nn_loader > > nn_loader( m, "nn_loader",
        "Device that accepts NN model packages (.hkrp files from the Perception Service)" );
    nn_loader.def( py::init< rs2::device >(), "device"_a )
        .def( "nn_upload",
              []( const rs2::nn_loader & self, py::buffer data, rs2_nn_target target, int slot, bool activate,
                  std::function< void( float ) > callback )
              {
                  py::buffer_info info = data.request();
                  if( info.strides.size() != 1 || info.strides[0] != info.itemsize )
                      throw std::runtime_error( "data must be a contiguous bytes-like object" );
                  auto begin = static_cast< const uint8_t * >( info.ptr );
                  std::vector< uint8_t > file( begin, begin + info.size * info.itemsize );
                  py::gil_scoped_release release;
                  nn_upload( self, file, target, slot, activate, std::move( callback ) );
              },
              "Upload an NN model package (bytes), sent to the device as is. Stop the Perception sensor first, and "
              "reboot the device (hardware_reset) afterwards for the new model to take effect. Currently only "
              "target=flash, slot=0 and activate=True are supported. The callback receives the progress in [0, 1].",
              "data"_a, "target"_a = RS2_NN_TARGET_FLASH, "slot"_a = 0, "activate"_a = true, "callback"_a = nullptr )
        .def( "nn_upload",
              []( const rs2::nn_loader & self, const std::string & path, rs2_nn_target target, int slot, bool activate,
                  std::function< void( float ) > callback )
              {
                  std::ifstream f( path, std::ios::binary );
                  if( ! f )
                      throw std::runtime_error( "cannot open " + path );
                  std::vector< uint8_t > file( ( std::istreambuf_iterator< char >( f ) ), std::istreambuf_iterator< char >() );
                  nn_upload( self, file, target, slot, activate, std::move( callback ) );
              },
              "Upload an NN model package from a file path; see the bytes overload.",
              "path"_a, "target"_a = RS2_NN_TARGET_FLASH, "slot"_a = 0, "activate"_a = true, "callback"_a = nullptr,
              py::call_guard< py::gil_scoped_release >() )
        .def( "is_upload_supported",
              []( const rs2::nn_loader & self )
              {
                  std::string reason;
                  bool supported = self.is_upload_supported( &reason );
                  return std::make_tuple( supported, reason );
              },
              "Whether the device can accept an upload now. Returns (supported, reason).",
              py::call_guard< py::gil_scoped_release >() );
}
