#ifndef __KINECT__
#define __KINECT__

#include <vector>

#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <cubemos/skeleton_tracking.h>

#include "util.hpp"

class kinect
{
private:
    // Kinect
    k4a::device device;
    k4a::capture capture;
    k4a::calibration calibration;
    k4a::transformation transformation;
    k4a_device_configuration_t device_configuration;
    uint32_t device_index;

    // Color
    k4a::image color_image;
    cv::Mat color;

    // Depth
    k4a::image depth_image;

    // Transformed
    k4a::image transformed_depth_image;

    // Cubemos
    CM_SKEL_Handle* handle;
    CM_SKEL_AsyncRequestHandle* request_handle;
    CUBEMOS_SKEL_Buffer_Ptr buffer;
    CUBEMOS_SKEL_Buffer_Ptr previous_buffer;

    // Visualize
    std::vector<cv::Scalar> colors;

public:
    // Constructor
    kinect( const uint32_t index = K4A_DEVICE_DEFAULT );

    // Destructor
    ~kinect();

    // Run
    void run();

    // Update
    void update();

    // Draw
    void draw();

    // Show
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

    // Update Transformation
    void update_transformation();

    // Update Skeleton
    void update_skeleton();

    // Draw Color
    void draw_color();

    // Draw Skeleton
    void draw_skeleton();

    // Show Skeleton
    void show_skeleton();
};

#endif // __KINECT__
