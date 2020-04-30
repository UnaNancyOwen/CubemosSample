#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <filesystem>
namespace filesystem = std::filesystem;

#include <opencv2/opencv.hpp>
#include <cubemos/skeleton_tracking.h>

#include "util.hpp"

int main( int argc, char* argv[] )
{
    try{
        // Open Capture
        cv::VideoCapture capture = cv::VideoCapture( 0 );
        if( !capture.isOpened() ){
            throw std::runtime_error( "failed to open!" );
        }

        // Set Capture Frame Resolution
        capture.set( cv::CAP_PROP_FRAME_WIDTH, 1280 );
        capture.set( cv::CAP_PROP_FRAME_HEIGHT, 720 );

        // Create Handle
        CM_SKEL_Handle* handle = nullptr;
        const filesystem::path license_directory( std::string( std::getenv( "LOCALAPPDATA" ) ) + "/Cubemos/SkeletonTracking/license" );
        CHECK_SUCCESS( cm_skel_create_handle( &handle, license_directory.generic_string().c_str() ) );

        // Load Model
        CM_TargetComputeDevice target_device = CM_TargetComputeDevice::CM_CPU;
        const filesystem::path model_directory( std::string( std::getenv( "LOCALAPPDATA" ) ) + "/Cubemos/SkeletonTracking/models" );
        const filesystem::path model( model_directory.generic_string() + "/fp32/skeleton-tracking.cubemos" ); // FP32 model
        //const filesystem::path model( model_directory.generic_string() + "/fp16/skeleton-tracking.cubemos" ); // FP16 model
        CHECK_SUCCESS( cm_skel_load_model( handle, target_device, model.generic_string().c_str() ) );

        // Create Async Request Handle
        CM_SKEL_AsyncRequestHandle* request_handle = nullptr;
        CHECK_SUCCESS( cm_skel_create_async_request_handle( handle, &request_handle ) );

        // Create Buffer
        CUBEMOS_SKEL_Buffer_Ptr buffer = create_skel_buffer();
        CUBEMOS_SKEL_Buffer_Ptr previous_buffer = create_skel_buffer();

        // Create Color Table
        std::vector<cv::Scalar> colors;
        colors.push_back( cv::Scalar( 255, 0, 0 ) );
        colors.push_back( cv::Scalar( 0, 255, 0 ) );
        colors.push_back( cv::Scalar( 0, 0, 255 ) );
        colors.push_back( cv::Scalar( 255, 255, 0 ) );
        colors.push_back( cv::Scalar( 0, 255, 255 ) );
        colors.push_back( cv::Scalar( 255, 0, 255 ) );

        while( true ){
            cv::Mat frame;
            capture >> frame;
            if( frame.empty() ){
                cv::waitKey( 0 );
                break;
            }
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

            //// Sync Inference
            //constexpr int32_t size = MULTIPLE * 8; // 16 * n
            //CM_ReturnCode result = cm_skel_estimate_keypoints( handle, &image, size, buffer.get() );

            // Async Inference
            constexpr int32_t size = MULTIPLE * 12; // 16 * n
            const std::chrono::milliseconds timeout( 1000 );
            CHECK_SUCCESS( cm_skel_estimate_keypoints_start_async( handle, request_handle, &image, size ) );
            CM_ReturnCode result = cm_skel_wait_for_keypoints( handle, request_handle, buffer.get(), timeout.count() );

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

                        constexpr int32_t radius = 5;
                        const cv::Point point = cv::Point( skeleton.keypoints_coord_x[j], skeleton.keypoints_coord_y[j] );
                        const cv::Scalar color = colors[skeleton.id % colors.size()];
                        cv::circle( frame, point, radius, color, -1, cv::LineTypes::LINE_AA );
                    }
                }

                // Swap and Release Previous Buffer
                previous_buffer.swap( buffer );
                cm_skel_release_buffer( buffer.get() );
            }

            // Show Image
            cv::imshow( "skeleton", frame );
            int32_t key = cv::waitKey( 10 );
            if( key == 'q' ){
                break;
            }
        }

        // Dstroy Handle and Window
        cm_skel_destroy_handle( &handle );
        cm_skel_destroy_async_request_handle( &request_handle );
        cv::destroyAllWindows();
    }
    catch( const std::runtime_error& error ){
        std::cout << error.what() << std::endl;
        return -1;
    }

    return 0;
}