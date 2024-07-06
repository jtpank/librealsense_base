#include "FrameProcessor.hpp"

FrameProcessor::FrameProcessor(unsigned int poolSize) 
: m_pOrb(cv::ORB::create()), m_bfMatcher(cv::BFMatcher::create(cv::NORM_HAMMING)), 
m_poolSize(poolSize), m_frameBuffer(poolSize), m_hasFirstFrame(false)
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

                std::cout << " threadId: " << threadId << " [ " << profile.stream_name() << " fnum: " << fnum << " ts: " << ts << "]\n";
                //" dt: " << dt[profile.stream_type()] << "] \n";
            }
            
            //Get the frames
            rs2::frame color_frame = aligned_frames.get_color_frame();
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
            cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat output_frame;
            orbDetectAndCompute(color_image, output_frame);
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
    // std::cout << "Corners length: " << corners.size() << std::endl;


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

void FrameProcessor::orbDetectAndCompute(cv::Mat &inputFrame, cv::Mat &outputFrame)
{
    cv::Mat des;
    cv::Mat grayFrame;
    inputFrame.copyTo(outputFrame);
    cv::cvtColor(outputFrame, grayFrame, cv::COLOR_BGR2GRAY);
    cv::Mat mask = cv::Mat::ones(grayFrame.size(), CV_8UC1) * 255;
    std::vector<cv::KeyPoint> kps;
    m_pOrb->detectAndCompute(grayFrame, mask, kps, des);
    cv::drawKeypoints(inputFrame, kps, outputFrame, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

    m_kps.push_back(kps);
    m_des.push_back(des);
    
}

void FrameProcessor::frameMatcher()
{
    //knn match
    //https://arxiv.org/pdf/2203.15119
    if(m_des.size() == 2)
    {
        assert(m_des.size() == 2);   
        std::vector<std::vector<cv::DMatch>> matches;
        m_bfMatcher->knnMatch(m_des.front(), m_des.back(), matches, 2);

        std::vector<cv::DMatch> good_matches;
        std::vector<cv::Point2f> srcPoints, dstPoints;
        if(matches.size() >= 2)
        {
            for(auto &match : matches)
            {
                if(match[0].distance < 0.75f * match[1].distance)
                {    
                    good_matches.emplace_back(match[0]);

                    // cv::Point2f srcPoint, dstPoint;
                    // srcPoint = m_kps.front().pt
                    // dstPoint = m_kps.back().pt;
                    srcPoints.emplace_back((m_kps.front())[match[0].queryIdx].pt);
                    dstPoints.emplace_back((m_kps.back())[match[0].trainIdx].pt);
                }
            }
            //so we only have 2
            m_des.pop_front();
            m_kps.pop_front();
        }

        // cv::Mat H = cv::findHomography(srcPoints, dstPoints, cv::RANSAC);
        // this->poseFromHomography(H);

        
    }
    else
    {
        std::cout << "not matching\n";
    }
}

void FrameProcessor::poseFromHomography(cv::Mat &H)
{
    //From: https://docs.opencv.org/3.4/d0/d92/samples_2cpp_2tutorial_code_2features2D_2Homography_2pose_from_homography_8cpp-example.html#a16
    cv::Mat R(3, 3, CV_64F);
    cv::Mat_<double> W, U, Vt;
    cv::SVDecomp(R, W, U, Vt);
    R = U*Vt;
    double det = determinant(R);
    if (det < 0)
    {
        Vt.at<double>(2,0) *= -1;
        Vt.at<double>(2,1) *= -1;
        Vt.at<double>(2,2) *= -1;
        
        R = U*Vt;
    }
    std::cout << "R (after polar decomposition):\n" << R << "\ndet(R): " << determinant(R) << std::endl;

    return;
}