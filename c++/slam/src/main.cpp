// include OpenCV header file
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "FrameProcessor.hpp"
#include <memory>
#include <cstdio>
using namespace std;



//TODO:
//1. Grab depth frame, vis frame, gyro, and accel frame
//2. Process these frames
//3.



int main()
{
    //Testing fp_ptr
    std::unique_ptr<FrameProcessor> fp_ptr = std::make_unique<FrameProcessor>();

    // fp_ptr->test_wrapGoodFeatures();


    bool doPipeline = true;
    if(doPipeline)
    {
        // Declare the RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;
        rs2::colorizer color_map;
        rs2::align align_to(RS2_STREAM_COLOR);
        //Create a configuration for configuring the pipeline with a non default profile
        rs2::config cfg;

        //Add desired streams to configuration
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        //Instruct pipeline to start streaming with the requested configuration
        pipe.start(cfg);

        //Display time
        const auto window_name = "Display Image";
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

        while (cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
        {
            // Camera warmup - dropping several first frames to let auto-exposure stabilize
            rs2::frameset frames;
            try {
                frames = pipe.wait_for_frames();
            } catch (const rs2::error & e) {
                std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n" << e.what() << std::endl;
                continue;
            }
            rs2::frameset aligned_frames = align_to.process(frames);

            //Get the frames
            rs2::frame color_frame = aligned_frames.get_color_frame();
            rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();

            printf("frames length: %li, aligned_frames length: %li\n", frames.size(), aligned_frames.size());

            if (!aligned_depth_frame  || !color_frame) 
            {
                continue;
            }
            // Creating OpenCV matrix for image
            cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);

            cv::Mat depth_colormap;
            depth_image.convertTo(depth_colormap, CV_8UC1, 0.03);
            cv::applyColorMap(depth_colormap, depth_colormap, cv::COLORMAP_JET);

            // Concatenate color and depth frames horizontally
            cv::Mat both_images;
            cv::hconcat(color_image, depth_colormap, both_images);

            cv::imshow(window_name, both_images);
        }
    }
    return 0;
}
