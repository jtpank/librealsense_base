// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API


bool check_imu_is_supported()
{
    bool found_gyro = false;
    bool found_accel = false;
    rs2::context ctx;
    for (auto dev : ctx.query_devices())
    {
        // The same device should support gyro and accel
        found_gyro = false;
        found_accel = false;
        for (auto sensor : dev.query_sensors())
        {
            for (auto profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == RS2_STREAM_GYRO)
                    found_gyro = true;

                if (profile.stream_type() == RS2_STREAM_ACCEL)
                    found_accel = true;
            }
        }
        if (found_gyro && found_accel)
            break;
    }
    return found_gyro && found_accel;
}

int main(int argc, char * argv[]) try
{
    // Before running the example, check that a device supporting IMU is connected
    if (!check_imu_is_supported())
    {
        std::cerr << "Device supporting IMU not found";
        return EXIT_FAILURE;
    }

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    //alignment
    rs2::align align_to(RS2_STREAM_COLOR);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add streams of gyro and accelerometer to configuration
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    // Start streaming with default recommended configuration
     rs2::pipeline_profile profile = pipe.start();

         // Get the color sensor
    auto sensor = profile.get_device().first<rs2::color_sensor>();

    // Disable Auto White Balance
    if (sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)) {
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);
    }

    // Set Manual White Balance to 4800K
    if (sensor.supports(RS2_OPTION_WHITE_BALANCE)) {
        sensor.set_option(RS2_OPTION_WHITE_BALANCE, 4800);
    }
    
    
    const auto window_name = "Display Image";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    while (cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
    {
        // rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        // rs2::frame depth = data.get_depth_frame().apply_filter(color_map);

        // // Query frame size (width and height)
        // const int w = depth.as<rs2::video_frame>().get_width();
        // const int h = depth.as<rs2::video_frame>().get_height();

        // // Create OpenCV matrix of size (w,h) from the colorized depth data
        // cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        // // Update the window with new data
        // cv::imshow(window_name, image);
        // Wait for the next set of frames
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frameset aligned_frames = align_to.process(frames);
        rs2::frame color_frame = aligned_frames.get_color_frame();
        rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();

        // Check if both frames are valid
        if (!color_frame || !aligned_depth_frame) {
            continue;
        }

        // Convert RealSense frames to OpenCV matrices
        cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);

        // Apply a colormap to the depth image
        cv::Mat depth_colormap;
        depth_image.convertTo(depth_colormap, CV_8UC1, 0.03);
        cv::applyColorMap(depth_colormap, depth_colormap, cv::COLORMAP_JET);

        // Wrap good features (assuming fpr.wrap_goodFeats is a custom function)
        // cv::Mat colorFrame, depthFrame;
        // fpr.wrap_goodFeats(color_image, depth_colormap, depth_image, colorFrame, depthFrame);

        // Concatenate color and depth frames horizontally
        cv::Mat both_images;
        cv::hconcat(color_image, depth_colormap, both_images);

        cv::imshow("test show vid", both_images);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
