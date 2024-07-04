#include "FrameProcessor.hpp"

FrameProcessor::FrameProcessor(unsigned int poolSize) : m_pOrb(cv::ORB::create()), m_poolSize(poolSize), m_frameBuffer(poolSize)
{   
    std::cout << "\nSetting up thread pool: " << std::endl;
    m_poolSize = poolSize;
    for(auto i = 0; i < poolSize; ++i)
    {
        //this leads to terminate called without an active exception
        // Aborted (core dumped)
        try {
             m_pool.emplace_back(std::thread( [=]() { frameConsumer(i); }));
        } catch (const std::exception& e) {
            std::cerr << "Exception during thread creation: " << e.what() << std::endl;
            throw;
        }
    }
}

void FrameProcessor::frameConsumer(int threadId)
{
    while(true)
    {
        try{
            rs2::frameset aligned_frames = m_frameBuffer.pop();
            //From: https://github.com/GruffyPuffy/imutest/blob/master/imutest.cpp
            for (auto f : aligned_frames)
            {
                rs2::stream_profile profile = f.get_profile();

                unsigned long fnum = f.get_frame_number();
                double ts = f.get_timestamp();
                // dt[profile.stream_type()] = (ts - last_ts[profile.stream_type()] ) / 1000.0;
                // last_ts[profile.stream_type()] = ts;

                // std::cout << " threadId: " << threadId << " [ " << profile.stream_name() << " fnum: " << fnum << " ts: " << ts << "]\n";
                //" dt: " << dt[profile.stream_type()] << "] \n";
            }
            
            //Get the frames
            // rs2::frame color_frame = aligned_frames.get_color_frame();
            rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
            rs2::frame accel_frame = aligned_frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
            rs2::motion_frame accel = accel_frame.as<rs2::motion_frame>();
            rs2::frame gyro_frame = aligned_frames.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
            rs2::motion_frame gyro = gyro_frame.as<rs2::motion_frame>();
            //helper function: auto depth_mat = depth_frame_to_meters(pipe, depth_frame);
            // printf("frames length: %li, aligned_frames length: %li\n", frames.size(), aligned_frames.size());

            if (!aligned_depth_frame ) // || !color_frame) 
            {
                continue;
            }

            if (accel)
            {
                rs2_vector av = accel.get_motion_data();
                float R = sqrtf(av.x * av.x + av.y * av.y + av.z * av.z);
                float newRoll = acos(av.x / R);
                float newYaw = acos(av.y / R);
                float newPitch = acos(av.z / R);
                // std::cout << "accX=" << newRoll << " accY=" << newYaw << " accZ=" << newPitch << std::endl;
            }
            if (gyro)
            {
                rs2_vector gv = gyro.get_motion_data();
                float gvx   = gv.x;
                float gvy    = gv.y;
                float gvz  = gv.z;
                // std::cout << "gvx=" << gvx << " gvy=" << gvy << " gvz=" << gvz << std::endl;
            }
        }
        catch(const std::runtime_error &e){
        }
    }
}

void FrameProcessor::processFrameset(rs2::frameset& frameSet)
{
    m_frameBuffer.push(frameSet);
}

void FrameProcessor::set_depthScale(float depthScale) 
{
    m_depthScale = depthScale;
}
void FrameProcessor::wrapGoodFeatures(cv::Mat &inputFrame, cv::Mat &outputFrame) 
{
    cv::Mat grayFrame;
    std::vector<cv::Point2f> corners;
    inputFrame.copyTo(outputFrame);
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
    m_pOrb->compute(inputFrame, kps1, des1);
    
    // cv::Mat des1;
    // this->m_pOrb->compute(inputFrame, kps1, des1);
    std::cout << "Keypoints Size: " << kps1.size() << std::endl;

    //Drawing the features
    int radius = 2;
    for(auto &corner : corners)
    {
        cv::circle(outputFrame, corner, radius, cv::Scalar(0, 255, 0));
    }

    return;
}
void FrameProcessor::test_wrapGoodFeatures()
{   
    //TODO: this returns a cv::Exception if not called from the directory with the image
    cv::Mat inputFrame = cv::imread("./test-image.png", cv::IMREAD_COLOR);
    cv::Mat outputFrame;

    wrapGoodFeatures(inputFrame, outputFrame);
    const auto goodFeatsWindow = "Good Features Image"; 
    cv::namedWindow(goodFeatsWindow, cv::WINDOW_AUTOSIZE);

    while(cv::waitKey(1))
    {
        cv::imshow(goodFeatsWindow, outputFrame);
    }

}