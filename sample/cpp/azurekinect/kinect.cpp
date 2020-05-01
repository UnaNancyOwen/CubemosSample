#include "kinect.hpp"

#include <array>
#include <chrono>
#include <vector>
#include <string>
#include <filesystem>
namespace filesystem = std::filesystem;

// Constructor
kinect::kinect( const uint32_t index )
    : device_index( index ),
      handle( nullptr ),
      request_handle( nullptr ),
      buffer( create_skel_buffer() ),
      previous_buffer( create_skel_buffer() )
{
    // Initialize
    initialize();
}

kinect::~kinect()
{
    // Finalize
    finalize();
}

// Initialize
void kinect::initialize()
{
    // Initialize Sensor
    initialize_sensor();

    // Initialize Skeleton
    initialize_skeleton();
}

// Initialize Sensor
inline void kinect::initialize_sensor()
{
    // Get Connected Devices
    const int32_t device_count = k4a::device::get_installed_count();
    if( device_count == 0 ){
        throw k4a::error( "Failed to found device!" );
    }

    // Open Default Device
    device = k4a::device::open( device_index );

    // Start Cameras with Configuration
    device_configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_configuration.color_format             = k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32;
    device_configuration.color_resolution         = k4a_color_resolution_t::K4A_COLOR_RESOLUTION_720P;
    device_configuration.depth_mode               = k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED;
    device_configuration.synchronized_images_only = true;
    device_configuration.wired_sync_mode          = k4a_wired_sync_mode_t::K4A_WIRED_SYNC_MODE_STANDALONE;
    device.start_cameras( &device_configuration );

    // Get Calibration
    calibration = device.get_calibration( device_configuration.depth_mode, device_configuration.color_resolution );

    // Create Transformation
    transformation = k4a::transformation( calibration );
}

// Initialize Skeleton
inline void kinect::initialize_skeleton()
{
    // Create Handle
    const filesystem::path license_directory( std::string( std::getenv( "LOCALAPPDATA" ) ) + "/Cubemos/SkeletonTracking/license" );
    CHECK_SUCCESS( cm_skel_create_handle( &handle, license_directory.generic_string().c_str() ) );

    // Load Model
    const CM_TargetComputeDevice target_device = CM_TargetComputeDevice::CM_CPU;
    const filesystem::path model_directory( std::string( std::getenv( "LOCALAPPDATA" ) ) + "/Cubemos/SkeletonTracking/models" );
    const filesystem::path model( model_directory.generic_string() + "/fp32/skeleton-tracking.cubemos" ); // FP32 model
    //const filesystem::path model( model_directory.generic_string() + "/fp16/skeleton-tracking.cubemos" ); // FP16 model
    CHECK_SUCCESS( cm_skel_load_model( handle, target_device, model.generic_string().c_str() ) );

    // Create Async Request Handle
    CHECK_SUCCESS( cm_skel_create_async_request_handle( handle, &request_handle ) );

    // Create Color Table
    colors.push_back( cv::Scalar( 255, 0, 0 ) );
    colors.push_back( cv::Scalar( 0, 255, 0 ) );
    colors.push_back( cv::Scalar( 0, 0, 255 ) );
    colors.push_back( cv::Scalar( 255, 255, 0 ) );
    colors.push_back( cv::Scalar( 0, 255, 255 ) );
    colors.push_back( cv::Scalar( 255, 0, 255 ) );
}

// Finalize
void kinect::finalize()
{
    // Dstroy Handle and Window
    if( handle != nullptr ){
        cm_skel_destroy_handle( &handle );
    }
    if( request_handle != nullptr ){
        cm_skel_destroy_async_request_handle( &request_handle );
    }

    // Destroy Transformation
    transformation.destroy();

    // Stop Cameras
    device.stop_cameras();

    // Close Device
    device.close();

    // Close Window
    cv::destroyAllWindows();
}

// Run
void kinect::run()
{
    // Main Loop
    while( true ){
        // Update
        update();

        // Draw
        draw();

        // Show
        show();

        // Wait Key
        constexpr int32_t delay = 10;
        const int32_t key = cv::waitKey( delay );
        if( key == 'q' ){
            break;
        }
    }
}

// Update
void kinect::update()
{
    // Update Frame
    update_frame();

    // Update Color
    update_color();

    // Update Depth
    update_depth();

    // Update Transformation
    update_transformation();

    // Update Skeleton
    update_skeleton();

    // Release Capture Handle
    capture.reset();
}

// Update Frame
inline void kinect::update_frame()
{
    // Get Capture Frame
    constexpr std::chrono::milliseconds time_out( K4A_WAIT_INFINITE );
    const bool result = device.get_capture( &capture, time_out );
    if( !result ){
        this->~kinect();
    }
}

// Update Color
inline void kinect::update_color()
{
    // Get Color Image
    color_image = capture.get_color_image();
}

// Update Depth
inline void kinect::update_depth()
{
    // Get Depth Image
    depth_image = capture.get_depth_image();
}

// Update Transformation
void kinect::update_transformation()
{
    if( !color_image.handle() || !depth_image.handle() ){
        return;
    }

    // Transform Depth Image to Color Camera
    transformed_depth_image = transformation.depth_image_to_color_camera( depth_image );
}

// Update Skeleton
inline void kinect::update_skeleton()
{
    if( !color_image.handle() ){
        return;
    }

    // Get cv::Mat from k4a::image
    cv::Mat frame = k4a::get_mat( color_image );

    // Only Support 3-channels Image
    if( frame.channels() == 4 ){
        cv::cvtColor( frame, frame, cv::COLOR_BGRA2BGR );
    }

    // Create Image
    CM_Image image = CM_Image{
        reinterpret_cast<void*>( frame.data ),
        CM_Datatype::CM_UINT8,
        frame.cols,
        frame.rows,
        frame.channels(),
        static_cast<int32_t>( frame.step[0] ),
        CM_MemoryOrder::CM_HWC
    };

    // Async Inference
    constexpr int32_t size = MULTIPLE * 12; // 16 * n
    CHECK_SUCCESS( cm_skel_estimate_keypoints_start_async( handle, request_handle, &image, size ) );
}

// Draw
void kinect::draw()
{
    // Draw Color
    draw_color();

    // Draw Skeleton
    draw_skeleton();
}

// Draw Color
inline void kinect::draw_color()
{
    if( !color_image.handle() ){
        return;
    }

    // Get cv::Mat from k4a::image
    color = k4a::get_mat( color_image );

    // Release Color Image Handle
    color_image.reset();
}

// Draw Skeleton
inline void kinect::draw_skeleton()
{
    if( color.empty() ){
        return;
    }

    if( !transformed_depth_image.handle() ){
        return;
    }

    // Get Inference Result
    const std::chrono::milliseconds timeout( 1000 );
    const CM_ReturnCode result = cm_skel_wait_for_keypoints( handle, request_handle, buffer.get(), timeout.count() );

    // Draw Skeleton
    if( result == CM_ReturnCode::CM_SUCCESS ){
        // Update Tracking ID
        CHECK_SUCCESS( cm_skel_update_tracking_id( handle, previous_buffer.get(), buffer.get() ) );

        // Draw Skeleton
        uint16_t* transformed_depth = reinterpret_cast<uint16_t*>( transformed_depth_image.get_buffer() );
        for( int32_t i = 0; i < buffer->numSkeletons; i++ ){
            const CM_SKEL_KeypointsBuffer& skeleton = buffer->skeletons[i];
            for( int32_t j = 0; j < skeleton.numKeyPoints; j++ ){
                constexpr float threshold = 0.5f;
                if( skeleton.confidences[j] < threshold ){
                    continue;
                }

                // Draw Joint
                constexpr int32_t radius = 5;
                const cv::Point point = cv::Point( skeleton.keypoints_coord_x[j], skeleton.keypoints_coord_y[j] );
                const cv::Scalar color = colors[skeleton.id % colors.size()];
                cv::circle( this->color, point, radius, colors[skeleton.id % colors.size()], -1, cv::LineTypes::LINE_AA );

                // Get 3D Position
                k4a_float3_t point_3d;
                const k4a_float2_t point_2d = { point.x, point.y };
                const bool result = calibration.convert_2d_to_3d( point_2d, transformed_depth[point.y * point.x], k4a_calibration_type_t::K4A_CALIBRATION_TYPE_COLOR, k4a_calibration_type_t::K4A_CALIBRATION_TYPE_COLOR, &point_3d );
                if( !result ){
                    continue;
                }

                // Draw 3D Position
                constexpr int32_t offcet = 20;
                constexpr double scale = 0.5;
                const std::string label = cv::format( "( %8.2f, %8.2f, %8.2f )", point_3d.xyz.x, point_3d.xyz.y, point_3d.xyz.z );
                cv::putText( this->color, label, cv::Point( point.x - offcet, point.y - offcet ), cv::FONT_HERSHEY_COMPLEX, scale, color );
            }
        }

        // Swap and Release Previous Buffer
        previous_buffer.swap( buffer );
        cm_skel_release_buffer( buffer.get() );
    }
}

// Show
void kinect::show()
{
    // Show Skeleton
    show_skeleton();
}

// Show Color
inline void kinect::show_skeleton()
{
    if( color.empty() ){
        return;
    }

    // Show Image
    const cv::String window_name = cv::format( "skeleton (kinect %d)", device_index );
    cv::imshow( window_name, color );
}
