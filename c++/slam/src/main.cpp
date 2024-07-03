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

        //Create a configuration for configuring the pipeline with a non default profile
        rs2::config cfg;

        //Add desired streams to configuration
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F);
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

            printf("frames length: %i\n", frames.size());
            //Get each frame
            // rs2::frame ir_frame = frames.get_infrared_frame(1);
            // rs2::frame ir_frame_2 = frames.get_infrared_frame(2);
            rs2::frame depth_frame = frames.get_depth_frame();
            if (!depth_frame) 
            {
                continue;
            }
            // Creating OpenCV matrix from IR image
            cv::Mat ir(Size(640, 480), cv::CV_8UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            // Apply Histogram Equalization
            // equalizeHist( ir, ir );
            cv::applyColorMap(depth_frame, depth_frame, cv::COLORMAP_JET);
            cv::imshow(window_name, depth_frame);
        }
    }
    return 0;
}
