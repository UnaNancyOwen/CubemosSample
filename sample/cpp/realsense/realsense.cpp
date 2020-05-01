#include "realsense.hpp"

#include <array>
#include <chrono>
#include <vector>
#include <string>
#include <filesystem>
namespace filesystem = std::filesystem;

// Constructor
realsense::realsense()
    : handle( nullptr ),
      request_handle( nullptr),
      buffer( create_skel_buffer() ),
      previous_buffer( create_skel_buffer() )
{
    // Initialize
    initialize();
}

// Destructor
realsense::~realsense()
{
    // Finalize
    finalize();
}

// Processing
void realsense::run()
{
    // Main Loop
    while( true ){
        // Update Data
        update();

        // Draw Data
        draw();

        // Show Data
        show();

        // Key Check
        const int32_t key = cv::waitKey( 10 );
        if( key == 'q' ){
            break;
        }
    }
}

// Initialize
void realsense::initialize()
{
    cv::setUseOptimized( true );

    // Initialize Sensor
    initialize_sensor();

    // Initialize Skeleton
    initialize_skeleton();
}

// Initialize Sensor
inline void realsense::initialize_sensor()
{
    // Set Device Config
    rs2::config config;
    config.enable_stream( rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps );
    config.enable_stream( rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps );

    // Start Pipeline
    pipeline_profile = pipeline.start( config );

    // Get Intrinsics
    intrinsics = pipeline_profile.get_stream( rs2_stream::RS2_STREAM_DEPTH ).as<rs2::video_stream_profile>().get_intrinsics();
}

// Initialize Skeleton
void realsense::initialize_skeleton()
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
    colors.push_back( cv::Scalar( 255,   0,   0 ) );
    colors.push_back( cv::Scalar(   0, 255,   0 ) );
    colors.push_back( cv::Scalar(   0,   0, 255 ) );
    colors.push_back( cv::Scalar( 255, 255,   0 ) );
    colors.push_back( cv::Scalar(   0, 255, 255 ) );
    colors.push_back( cv::Scalar( 255,   0, 255 ) );
}

// Finalize
void realsense::finalize()
{
    // Dstroy Handle and Window
    if( handle != nullptr){
        cm_skel_destroy_handle( &handle );
    }
    if( request_handle != nullptr){
        cm_skel_destroy_async_request_handle( &request_handle );
    }

    // Stop Pipline
    pipeline.stop();

    // Close Windows
    cv::destroyAllWindows();
}

// Update Data
void realsense::update()
{
    // Update Frame
    update_frame();

    // Update Color
    update_color();

    // Update Depth
    update_depth();

    // Update Skeleton
    update_skeleton();
}

// Update Frame
inline void realsense::update_frame()
{
    // Update Frame
    frameset = pipeline.wait_for_frames();
}

// Update Color
inline void realsense::update_color()
{
    // Retrieve Color Frame
    color_frame = frameset.get_color_frame();

    // Retrive Frame Size
    color_width = color_frame.as<rs2::video_frame>().get_width();
    color_height = color_frame.as<rs2::video_frame>().get_height();
    color_stride = color_frame.as<rs2::video_frame>().get_stride_in_bytes();
}

// Update Depth
void realsense::update_depth()
{
    // Retrieve Depth Frame
    depth_frame = frameset.get_depth_frame();

    // Retrive Frame Size
    depth_width = depth_frame.as<rs2::video_frame>().get_width();
    depth_height = depth_frame.as<rs2::video_frame>().get_height();
}

// Update Skeleton
void realsense::update_skeleton()
{
    // Get Image
    cv::Mat frame;
    switch( color_frame.get_profile().format() ){
        case rs2_format::RS2_FORMAT_BGR8:
            frame = cv::Mat( color_height, color_width, CV_8UC3, const_cast<void*>( color_frame.get_data() ) ).clone();
            break;
        case rs2_format::RS2_FORMAT_RGBA8:
            frame = cv::Mat( color_height, color_width, CV_8UC4, const_cast<void*>( color_frame.get_data() ) ).clone();
            cv::cvtColor( frame, frame, cv::COLOR_BGRA2BGR );
            break;
        default:
            throw std::runtime_error( "this format not support!" );
            break;
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

// Draw Data
void realsense::draw()
{
    // Draw Color
    draw_color();

    // Draw Skeleton
    draw_skeleton();
}

// Draw Color
inline void realsense::draw_color()
{
    // Create cv::Mat form Color Frame
    switch( color_frame.get_profile().format() ){
        case rs2_format::RS2_FORMAT_BGR8:
            color = cv::Mat( color_height, color_width, CV_8UC3, const_cast<void*>( color_frame.get_data() ) );
            break;
        case rs2_format::RS2_FORMAT_RGBA8:
            color = cv::Mat( color_height, color_width, CV_8UC4, const_cast<void*>( color_frame.get_data() ) );
            break;
        default:
            throw std::runtime_error( "this format not support!" );
            break;
    }
}

// Draw Skeleton
inline void realsense::draw_skeleton()
{
    if( color.empty() ){
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
                std::array<float, 3> point_3d;
                const std::array<float, 2> point_2d = { point.x, point.y };
                const float distance = depth_frame.as<rs2::depth_frame>().get_distance( point.x, point.y );
                rs2_deproject_pixel_to_point( &point_3d[0], &intrinsics, &point_2d[0], distance );

                // Draw 3D Position
                constexpr int32_t offcet = 20;
                constexpr double scale = 0.5;
                const std::string label = cv::format( "( %8.2f, %8.2f, %8.2f )", point_3d[0], point_3d[1], point_3d[2] );
                cv::putText( this->color, label, cv::Point( point.x - offcet, point.y - offcet ), cv::FONT_HERSHEY_COMPLEX, scale, color );
            }
        }

        // Swap and Release Previous Buffer
        previous_buffer.swap( buffer );
        cm_skel_release_buffer( buffer.get() );
    }
}

// Show Data
void realsense::show()
{
    // Show Skeleton
    show_skeleton();
}

// Show Color
inline void realsense::show_skeleton()
{
    if( color.empty() ){
        return;
    }

    // Show Skeleton Image
    cv::imshow( "skeleton", color );
}