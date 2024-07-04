// include OpenCV header file
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "FrameProcessor.hpp"
#include "FrameBuffer.hpp"
#include <memory>
#include <thread>
#include <cstdio>

using namespace std;


#define CAPACITY 8


//TODO:
//1. Grab depth frame, vis frame, gyro, and accel frame
//2. Process these frames
//3.



int main()
{
    unsigned int n_threads = std::thread::hardware_concurrency();
    std::cout << n_threads << " concurrent threads supported." << std::endl;
    //Testing fp_ptr
    try {
        rs2::context ctx;
        auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
        if (list.size() == 0) 
            throw std::runtime_error("No device detected. Is it plugged in?");
        rs2::device dev = list.front();
        //https://github.com/IntelRealSense/librealsense/wiki/API-How-To#do-processing-on-a-background-thread

        // Declare the RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;
        rs2::colorizer color_map;
        //Create a configuration for configuring the pipeline with a non default profile
        rs2::config cfg;

        //Add desired streams to configuration
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        bool firstAccel = true;
        double last_ts[RS2_STREAM_COUNT];
        double dt[RS2_STREAM_COUNT];


        //Instruct pipeline to start streaming with the requested configuration
        rs2::pipeline_profile pipeline_profile = pipe.start(cfg);
        
        //Very important for aligning frames
        rs2::align align(RS2_STREAM_COLOR);
        //Display time
        const auto window_name = "Display Image";
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

        // while (cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)

        std::unique_ptr<FrameProcessor> fp_ptr = std::make_unique<FrameProcessor>(n_threads);
        while(cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
        {
            // Camera warmup - dropping several first frames to let auto-exposure stabilize
            rs2::frameset frames, aligned_frames;
            try {
                frames = pipe.wait_for_frames();
                aligned_frames = align.process(frames);
            } catch (const rs2::error & e) {
                std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n" << e.what() << std::endl;
                continue;
            }
            fp_ptr->processFrameset(aligned_frames);


            //replace the following in the frame processor method

            // Creating OpenCV matrix for image
            rs2::frame color_frame = aligned_frames.get_color_frame();
            cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            // cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat output_frame;
            fp_ptr->wrapGoodFeatures(color_image, output_frame);
            // cv::Mat depth_colormap;
            // depth_image.convertTo(depth_colormap, CV_8UC1, 0.03);
            // cv::applyColorMap(depth_colormap, depth_colormap, cv::COLORMAP_JET);

            // // Concatenate color and depth frames horizontally
            // cv::Mat both_images;
            // cv::hconcat(color_image, depth_colormap, both_images);

            cv::imshow(window_name, output_frame);
        }
    }
    catch (const rs2::error & e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception & e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    
    return 0;
}
