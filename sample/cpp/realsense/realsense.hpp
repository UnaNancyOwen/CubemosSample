#ifndef __REALSENSE__
#define __REALSENSE__

#include <vector>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <cubemos/skeleton_tracking.h>

#include "util.hpp"

class realsense
{
private:
    // RealSense
    rs2::pipeline pipeline;
    rs2::pipeline_profile pipeline_profile;
    rs2::frameset frameset;

    // Color
    rs2::frame color_frame;
    cv::Mat color;
    int32_t color_width  = 1280;
    int32_t color_height = 720;
    int32_t color_fps = 30;
    int32_t color_stride;

    // Depth
    rs2::frame depth_frame;
    rs2_intrinsics intrinsics;
    int32_t depth_width = 1280;
    int32_t depth_height = 720;
    int32_t depth_fps = 30;

    // Cubemos
    CM_SKEL_Handle* handle;
    CM_SKEL_AsyncRequestHandle* request_handle;
    CUBEMOS_SKEL_Buffer_Ptr buffer;
    CUBEMOS_SKEL_Buffer_Ptr previous_buffer;

    // Visualize
    std::vector<cv::Scalar> colors;

public:
    // Constructor
    realsense();

    // Destructor
    ~realsense();

    // Processing
    void run();

    // Update Data
    void update();

    // Draw Data
    void draw();

    // Show Data
    void show();

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    void initialize_sensor();

    // Initialize Skeleton
    void initialize_skeleton();

    // Finalize
    void finalize();

    // Update Frame
    void update_frame();

    // Update Color
    void update_color();

    // Update Depth
    void update_depth();

    // Update Skeleton
    void update_skeleton();

    // Draw Color
    void draw_color();

    // Draw Skeleton
    void draw_skeleton();

    // Show Skelton
    void show_skeleton();
};

#endif // __REALSENSE__