#include "FrameProcessor.hpp"

FrameProcessor::FrameProcessor(unsigned int poolSize) 
: m_pOrb(cv::ORB::create()), m_bfMatcher(cv::BFMatcher::create(cv::NORM_HAMMING)),
m_poolSize(poolSize), m_frameBuffer(poolSize, frameSetType), m_colorFrameBuffer(poolSize, colorFrameType), 
m_depthFrameBuffer(poolSize, depthFrameType), m_imuFrameBuffer(poolSize, imuFrameType),
m_hasFirstFrame(false),  m_shutdownThreads(false)
{   
    std::cout << "\nSetting up thread pool: " << std::endl;
    m_poolSize = poolSize;

    try {
        //Push back 3 threads for each frame
        m_pool.emplace_back(std::thread( [=]() { frameConsumer(0, colorFrameType); }));
        m_pool.emplace_back(std::thread( [=]() { frameConsumer(1, depthFrameType); }));
        m_pool.emplace_back(std::thread( [=]() { frameConsumer(2, imuFrameType); }));
        std::cout << "\tFinished setting up thread pool." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Exception during thread creation: " << e.what() << std::endl;
        throw;
    }
    
}

void FrameProcessor::frameConsumer(int threadId, FrameBufferType ftype)
{
    while(true && !this->m_shutdownThreads)
    {
        try {
            switch(ftype)
            {
                case colorFrameType:
                {
                    this->consumeColorFrame(threadId);
                    break;
                }
                case depthFrameType:
                {
                    this->consumeDepthFrame(threadId);
                    break;
                }
                case imuFrameType:
                {
                    this->consumeImuFrame(threadId);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
        catch(const std::runtime_error &e){
        }
    }
}


void FrameProcessor::framesetConsumer(int threadId)
{
    while(true && !this->m_shutdownThreads)
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

void FrameProcessor::consumeColorFrame(int threadId)
{   
    rs2::frame color_frame = m_colorFrameBuffer.pop();
    cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat output_frame;
    orbDetectAndCompute(color_image, output_frame);
}
void FrameProcessor::consumeDepthFrame(int threadId)
{
    rs2::frame depth_frame = m_depthFrameBuffer.pop();
    cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
}
void FrameProcessor::consumeImuFrame(int threadId)
{
    std::vector<rs2::frame> bothFrames = m_imuFrameBuffer.pop();
    rs2::frame accel_frame = bothFrames[0];
    rs2::frame gyro_frame = bothFrames[1];
    rs2::motion_frame accel = accel_frame.as<rs2::motion_frame>();
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
    
}

void FrameProcessor::joinAllThreads()
{
    this->m_shutdownThreads = false;
    for(auto &th : m_pool)
    {
        th.join();
    }
    std::cout << "Finished joining all threads." << std::endl;
}

void FrameProcessor::processFramesToIndividualBuffers(rs2::frameset& frameSet)
{
    rs2::frame color_frame = frameSet.get_color_frame();
    rs2::depth_frame depth_frame = frameSet.get_depth_frame();
    rs2::frame accel_frame = frameSet.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    rs2::frame gyro_frame = frameSet.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    std::vector<rs2::frame> imuFrames;
    imuFrames.emplace_back(accel_frame);
    imuFrames.emplace_back(gyro_frame);
    m_colorFrameBuffer.push(color_frame);
    m_depthFrameBuffer.push(depth_frame);
    m_imuFrameBuffer.push(imuFrames);
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
    // std::cout << "m_kps size: "<< m_kps.size() << std::endl;
    // std::cout << "m_des size: "<< m_des.size() << std::endl;
    assert(m_kps.size() == m_des.size());
    if(m_kps.size() > 2)
    {
        m_kps.pop_front();
        m_des.pop_front();
    }
    
}

void FrameProcessor::grabVertices(rs2::depth_frame &depth_frame, rs2::points &points, rs2::pointcloud &pc)
{
    points = pc.calculate(depth_frame);
    const rs2::vertex* vertices = points.get_vertices();
    m_vertices.push_back(vertices);
}

void FrameProcessor::frameMatcher()
{
    //knn match
    //https://arxiv.org/pdf/2203.15119
    if(m_des.size() == 2)
    {
        assert(m_des.size() == 2);   
        assert(m_vertices.size() == 2); 
        std::vector<std::vector<cv::DMatch>> matches;
        m_bfMatcher->knnMatch(m_des.front(), m_des.back(), matches, 2);

        std::vector<cv::DMatch> good_matches;
        std::vector<rs2::vertex> good_srcPoints; // 3-D x,y,z from pointcloud
        std::vector<rs2::vertex> good_dstPoints; // 3-D x,y,z from pointcloud
        std::vector<cv::Point2f> srcPoints, dstPoints;
        if(matches.size() >= 2)
        {
            for(auto &match : matches)
            {
                if(match[0].distance < 0.75f * match[1].distance)
                {    
                    good_matches.emplace_back(match[0]);

                    cv::Point2f srcPoint, dstPoint;
                    srcPoint = (m_kps.front())[match[0].queryIdx].pt;
                    dstPoint = (m_kps.back())[match[0].trainIdx].pt;
                    srcPoints.emplace_back(srcPoint);
                    dstPoints.emplace_back(dstPoint);

                    const rs2::vertex* queryVertices = m_vertices.back();
                    const rs2::vertex* trainVertices = m_vertices.front();
                    //TODO: Ensure this is correct (row major, I believe)
                    good_srcPoints.emplace_back(queryVertices[static_cast<int>(srcPoint.x)*480 + static_cast<int>(srcPoint.y)]);
                    good_dstPoints.emplace_back(trainVertices[static_cast<int>(dstPoint.x)*480 + static_cast<int>(dstPoint.y)]);
                    
                }
            }
        }
        //so we only have 2
        m_des.pop_front();
        m_kps.pop_front();
        m_vertices.pop_front();
        assert(good_srcPoints.size() == good_dstPoints.size());
        assert(good_matches.size() == good_srcPoints.size());
        std::cout << "size: " << good_srcPoints.size() << std::endl;

        //1. compute the centroids
        float3 good_srcCentroid, good_dstCentroid;
        good_srcCentroid.x = 0.f;
        good_srcCentroid.y = 0.f;
        good_srcCentroid.z = 0.f;
        good_dstCentroid.x = 0.f;
        good_dstCentroid.y = 0.f;
        good_dstCentroid.z = 0.f;
        for(int i = 0; i < good_srcPoints.size(); ++i)
        {
            good_srcCentroid.add(good_srcPoints[i].x, good_srcPoints[i].y, good_srcPoints[i].z);
            good_dstCentroid.add(good_dstPoints[i].x, good_dstPoints[i].y, good_dstPoints[i].z);
        }
        good_srcCentroid = good_srcCentroid / static_cast<float>(good_srcPoints.size());
        good_dstCentroid = good_dstCentroid / static_cast<float>(good_dstPoints.size());

        std::cout << "good_srcCentroid: x,y,z: " << good_srcCentroid.x << "," << good_srcCentroid.y << "," << good_srcCentroid.z << std::endl;
        std::cout << "good_dstCentroid: x,y,z: " << good_dstCentroid.x << "," << good_dstCentroid.y << "," << good_dstCentroid.z << std::endl;  
        
        //2. compute centralized vectors
        // std::vector<float3> cent_srcVector, cent_dstVector;
        // for(int i = 0; i < good_srcPoints.size(); ++i)
        // {
        //     float3 srcCent, dstCent;
        //     srcCent.x = good_srcPoints[i].x - good_srcCentroid.x;
        //     srcCent.y = good_srcPoints[i].y - good_srcCentroid.y;
        //     srcCent.z = good_srcPoints[i].z - good_srcCentroid.z;
        //     dstCent.x = good_dstPoints[i].x - good_dstCentroid.x;
        //     dstCent.y = good_dstPoints[i].y - good_dstCentroid.y;
        //     dstCent.z = good_dstPoints[i].z - good_dstCentroid.z;
        //     cent_srcVector.emplace_back(srcCent);
        //     cent_dstVector.emplace_back(dstCent);
        // }
        // //3. find covariance matrix S
        // cv::Mat srcMat = cv::Mat::zeros(3, cent_srcVector.size(),CV_32FC1);
        // cv::Mat dstMat = cv::Mat::zeros(3, cent_srcVector.size(),CV_32FC1);
        // cv::Mat dstTranspose;
        // for(int i = 0; i < cent_srcVector.size(); ++i)
        // {
        //     srcMat.at<double>(0, i) = cent_srcVector[i].x;
        //     srcMat.at<double>(1, i) = cent_srcVector[i].y;
        //     srcMat.at<double>(2, i) = cent_srcVector[i].z;

        //     dstMat.at<double>(0, i) = cent_dstVector[i].x;
        //     dstMat.at<double>(1, i) = cent_dstVector[i].y;
        //     dstMat.at<double>(2, i) = cent_dstVector[i].z;
        // }
        // cv::transpose(dstMat, dstTranspose);
        // cv::Mat covMat = srcMat * dstTranspose;
        // assert(covMat.rows == covMat.cols);
        // assert(covMat.rows == 3);
        //4. perform SVD.
        // cv::Mat_<double> w, u, vt;
        // cv::SVDecomp(covMat,w, u, vt);
        // std::cout << "CovMat: \n" << covMat << std::endl;
        //5. Output R_3x3 rotation matrix and tr_3x1 translation vector

        
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