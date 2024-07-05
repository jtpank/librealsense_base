// include OpenCV header file
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "FrameProcessor.hpp"
#include "FrameBuffer.hpp"
#include "RotationEstimator.hpp"
#include <memory>
#include <thread>
#include <cstdio>
#include <chrono>
#include "float3.hpp"
using namespace std;
#define CAPACITY 8


//If camera is facing out on a level surface
// normalized_accel = (0, -1, 0)
// If facing camera:
// +x is to my left
// +y is to my up
// +z is into the camer


int main()
{
    //Number of threads that can execute simultaneously
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

        //Instruct pipeline to start streaming with the requested configuration
        rs2::pipeline_profile pipeline_profile = pipe.start(cfg);
        
        //Very important for aligning frames
        rs2::align align(RS2_STREAM_COLOR);
        //Display time
        const auto windowName = "Depth and Color Images";
        cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

        //For gyro and accel pose
        double last_ts[RS2_STREAM_COUNT];
        double dt[RS2_STREAM_COUNT];
        std::unique_ptr<FrameProcessor> fp_ptr = std::make_unique<FrameProcessor>(n_threads);
        RotationEstimator algo;
        while(cv::waitKey(1) < 0 && cv::getWindowProperty(windowName, cv::WND_PROP_AUTOSIZE) >= 0)
        // while(true)
        {
            auto start = std::chrono::high_resolution_clock::now();

            // Camera warmup - dropping several first frames to let auto-exposure stabilize
            rs2::frameset frames, aligned_frames;
            try {
                frames = pipe.wait_for_frames();
                aligned_frames = align.process(frames);
            } catch (const rs2::error & e) {
                std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n" << e.what() << std::endl;
                continue;
            }
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start;
            // std::cout << "Grab frameset: Elapsed time: " << duration.count() << " ms " << std::endl;
            
            auto new_start = std::chrono::high_resolution_clock::now();        
            // for (auto f : aligned_frames)
            // {
            //     rs2::stream_profile profile = f.get_profile();
            //     unsigned long fnum = f.get_frame_number();
            //     double ts = f.get_timestamp();
            //     dt[profile.stream_type()] = (ts - last_ts[profile.stream_type()] ) / 1000.0;
            //     last_ts[profile.stream_type()] = ts;
            // }

            end = std::chrono::high_resolution_clock::now();
            duration = end - new_start;
            // std::cout << "Grab timestamps: Elapsed time: " << duration.count() << " ms " << std::endl;
            
            //Grab the frames
            new_start = std::chrono::high_resolution_clock::now();
            rs2::frame accel_frame = aligned_frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
            rs2::motion_frame accel = accel_frame.as<rs2::motion_frame>();
            rs2::frame gyro_frame = aligned_frames.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
            rs2::motion_frame gyro = gyro_frame.as<rs2::motion_frame>();
            double gyro_ts = gyro.get_timestamp();

            if (gyro)
            {
                rs2_vector gv = gyro.get_motion_data();
                algo.process_gyro(gv, gyro_ts);
            }
            if (accel)
            {
                rs2_vector av = accel.get_motion_data();
                algo.process_accel(av);
            }
            float3 outputTheta = (algo.get_theta())* 180.0 / M_PI;
            std::cout << "Pitch: " << outputTheta.x << " Yaw: " << outputTheta.y << " Roll: " << outputTheta.z << std::endl;

            rs2::frame color_frame = aligned_frames.get_color_frame();
            // rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
            cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            // cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat output_frame;
            // fp_ptr->wrapGoodFeatures(color_image, output_frame);
            fp_ptr->orbDetectAndCompute(color_image, output_frame);
            fp_ptr->bfMatchFrames();
            cv::imshow(windowName, output_frame);
            // Output the duration in milliseconds
            end = std::chrono::high_resolution_clock::now();
            duration = end - new_start;
            std::chrono::duration<double, std::milli> duration_final = end - start;
            // std::cout << "Process Frames: Elapsed time: " << duration.count() << " ms " << std::endl;
            // std::cout << "Total Elapsed time: " << duration_final.count() << " ms " << std::endl;
            
          

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



/*

//1. split the frames 
            for (auto f : aligned_frames)
            {
                rs2::stream_profile profile = f.get_profile();
                unsigned long fnum = f.get_frame_number();
                double ts = f.get_timestamp();
                dt[profile.stream_type()] = (ts - last_ts[profile.stream_type()] ) / 1000.0;
                last_ts[profile.stream_type()] = ts;
            }

            rs2::frame color_frame = aligned_frames.get_color_frame();
            rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
            rs2::frame accel_frame = aligned_frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
            rs2::motion_frame accel = accel_frame.as<rs2::motion_frame>();
            rs2::frame gyro_frame = aligned_frames.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
            rs2::motion_frame gyro = gyro_frame.as<rs2::motion_frame>();
            cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat outputFrame;
            fp_ptr->wrapGoodFeatures(color_image, outputFrame);
            cv::Mat depthColormap;
            depth_image.convertTo(depthColormap, CV_8UC1, 0.03);
            cv::applyColorMap(depthColormap, depthColormap, cv::COLORMAP_JET);

            cv::Mat depthColormap, outputFrame;
            std::thread colorThread( [&aligned_frames, &outputFrame, m_pOrb]() { 
                rs2::frame color_frame = aligned_frames.get_color_frame();
                if(color_frame)
                {  
                    cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
                    cv::Mat grayFrame;
                    std::vector<cv::Point2f> corners;
                    color_image.copyTo(outputFrame);
                    cv::cvtColor(outputFrame, grayFrame, cv::COLOR_BGR2GRAY);

                    //Parameters should move elswhere
                    int max_count = 100; 
                    double quality_level = 0.01; 
                    double min_distance = 3;
                    cv::goodFeaturesToTrack(grayFrame, corners, max_count, quality_level, min_distance);
                    std::cout << "Corners length: " << corners.size() << std::endl;

                    std::vector<cv::KeyPoint> kps1;
                    cv::Mat des1;
                    for(auto &corner : corners)
                    {
                        kps1.emplace_back(cv::KeyPoint(corner, 1.f));
                    }
                    m_pOrb->compute(color_image, kps1, des1);
                    
                    // cv::Mat des1;
                    // this->m_pOrb->compute(color_image, kps1, des1);
                    std::cout << "Keypoints Size: " << kps1.size() << std::endl;

                    //Drawing the features
                    int radius = 2;
                    for(auto &corner : corners)
                    {
                        cv::circle(outputFrame, corner, radius, cv::Scalar(0, 255, 0));
                    }

                }
            });
            std::thread depthThread( [&aligned_frames, &depthColormap]() { 
                rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
                if(aligned_depth_frame)
                {
                    // cv::Mat depth_colormap;
                    cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
                    depth_image.convertTo(depthColormap, CV_8UC1, 0.03);
                    cv::applyColorMap(depthColormap, depthColormap, cv::COLORMAP_JET);
                }
            });
            std::thread imuThread( [&aligned_frames, dt]() { 
                rs2::frame accel_frame = aligned_frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
                rs2::motion_frame accel = accel_frame.as<rs2::motion_frame>();
                rs2::frame gyro_frame = aligned_frames.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
                rs2::motion_frame gyro = gyro_frame.as<rs2::motion_frame>();
                if (accel)
                {
                    rs2_vector av = accel.get_motion_data();
                    float R         = sqrtf(av.x * av.x + av.y * av.y + av.z * av.z);
                    float newRoll   = acos(av.x / R);
                    float newYaw    = acos(av.y / R);
                    float newPitch  = acos(av.z / R);
                    // std::cout << "accX=" << newRoll << " accY=" << newYaw << " accZ=" << newPitch << std::endl;
                }
                if (gyro)
                {
                    rs2_vector gv = gyro.get_motion_data();
                    float gvx   = gv.x;
                    float gvy   = gv.y;
                    float gvz   = gv.z;
                    // std::cout << "gvx=" << gvx << " gvy=" << gvy << " gvz=" << gvz << std::endl;
                }
            });

            colorThread.join();
            depthThread.join();
            imuThread.join();




                //replace the following in the frame processor method

            // Creating OpenCV matrix for image
            // rs2::frame color_frame = aligned_frames.get_color_frame();
            // cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            // // cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
            // cv::Mat output_frame;
            // fp_ptr->wrapGoodFeatures(color_image, output_frame);
            // // cv::Mat depth_colormap;
            // // depth_image.convertTo(depth_colormap, CV_8UC1, 0.03);
            // // cv::applyColorMap(depth_colormap, depth_colormap, cv::COLORMAP_JET);

            // // // Concatenate color and depth frames horizontally
            // // cv::Mat both_images;
            // // cv::hconcat(color_image, depth_colormap, both_images);

            // cv::imshow(windowName, color_image);
            // cv::Mat bothImages;
            // cv::hconcat(outputFrame, depthColormap, bothImages);
            // cv::imshow(windowName, bothImages);
*/