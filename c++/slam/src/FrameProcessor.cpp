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
        //Push back 3 threads, 1 for each frame
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
    m_colorCounter++;
    rs2::frame color_frame = m_colorFrameBuffer.pop();
    cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat output_frame;
    orbDetectAndCompute(color_image, output_frame);
    if(m_colorCounter % 20 == 0)
    {
        m_depthCounter = 0;
        std::cout << "threadid: " << threadId << " | Orb detect performed" << std::endl;
    }

}
void FrameProcessor::consumeDepthFrame(int threadId)
{
    m_depthCounter++;
    rs2::frame depth_frame = m_depthFrameBuffer.pop();
    cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    // TODO: This errors: cannot bind non-const lvalue reference of type ‘rs2::depth_frame&’ to an rvalue of type ‘rs2::depth_frame’
    // grabVertices(depth_frame, m_points, m_pc);
    m_points = m_pc.calculate(depth_frame);
    const rs2::vertex* vertices = m_points.get_vertices();
    m_vertices.push_back(vertices);
    if(m_vertices.size() > 2)
    {
        m_vertices.pop_front();
    }
    if(m_depthCounter % 20 == 0)
    {
        m_depthCounter = 0;
        std::cout << "threadid: " << threadId << " | Vertices found" << std::endl;
    }

}
void FrameProcessor::consumeImuFrame(int threadId)
{
    m_imuCounter++;
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
    if(m_imuCounter % 20 == 0)
    {
        m_imuCounter = 0;
        float3 outputTheta = (algo.get_theta())* 180.0 / M_PI;
        std::cout << "threadid: " << threadId << " | Pitch: " << outputTheta.x << " Yaw: " << outputTheta.y << " Roll: " << outputTheta.z << std::endl;
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
    frameSet.keep();
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


//TODO: dont pass in these points and pc They are member variables!
void FrameProcessor::grabVertices(rs2::depth_frame &depth_frame, rs2::points &points, rs2::pointcloud &pc)
{
    points = pc.calculate(depth_frame);
    const rs2::vertex* vertices = points.get_vertices();
    m_vertices.push_back(vertices);
    if(m_vertices.size() > 2)
    {
        m_vertices.pop_front();
    }
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
        //We now have physical XYZ points from pointcloud, in the src and dst frames, that are the pixel coordinates of the
        // good matches found using the brute force knn match algorithm
        

        //1. compute the centroids
        cv::Point3f p_centroid(0.0f, 0.0f, 0.0f), q_centroid(0.0f, 0.0f, 0.0f);
        int sz_n = good_srcPoints.size();
        for(int i = 0; i < sz_n; ++i)
        {
            p_centroid.x += good_srcPoints[i].x;
            p_centroid.y += good_srcPoints[i].y;
            p_centroid.z += good_srcPoints[i].z;
            
            q_centroid.x += good_dstPoints[i].x;
            q_centroid.y += good_dstPoints[i].y;
            q_centroid.z += good_dstPoints[i].z;
        }
        p_centroid.x /= static_cast<float>(sz_n);
        p_centroid.y /= static_cast<float>(sz_n);
        p_centroid.z /= static_cast<float>(sz_n);

        q_centroid.x /= static_cast<float>(sz_n);
        q_centroid.y /= static_cast<float>(sz_n);
        q_centroid.z /= static_cast<float>(sz_n);
        cv::Mat p_centMat = (cv::Mat_<float>(3,1) << p_centroid.x, p_centroid.y, p_centroid.z);
        cv::Mat q_centMat = (cv::Mat_<float>(3,1) << q_centroid.x, q_centroid.y, q_centroid.z);
        std::vector<cv::Point3f> vp, vq;
        for(int i = 0; i < sz_n; ++i)
        {
            cv::Point3f tempp, tempq;
            tempp.x = good_srcPoints[i].x - p_centroid.x;
            tempp.y = good_srcPoints[i].y - p_centroid.y;
            tempp.z = good_srcPoints[i].z - p_centroid.z;

            tempq.x = good_dstPoints[i].x - q_centroid.x;
            tempq.y = good_dstPoints[i].y - q_centroid.y;
            tempq.z = good_dstPoints[i].z - q_centroid.z;

            vp.emplace_back(tempp);
            vq.emplace_back(tempq);
        }
        // //3. find covariance matrix S
        cv::Mat mat_vp, mat_vq;
        for(int i = 0; i < sz_n; ++i)
        {
            cv::Mat tempvp = (cv::Mat_<float>(3,1) << vp[i].x, vp[i].y, vp[i].z);
            cv::Mat tempvq = (cv::Mat_<float>(1,3) << vq[i].x, vq[i].y, vq[i].z);
            if(mat_vp.empty())
                mat_vp = tempvp;
            else
                cv::hconcat(mat_vp, tempvp, mat_vp);
            if(mat_vq.empty())
                mat_vq = tempvq;
            else
                cv::vconcat(mat_vq, tempvq, mat_vq);
        }

        // covariance matrix
        cv::Mat s_mat;
        s_mat = mat_vp * mat_vq;
        assert(s_mat.rows == 3 && s_mat.cols == 3);
        //4. perform SVD.
        cv::Mat w, u, vt, v, ut;
        //vt is 3x3
        //u is 3x3
        m_svd.compute(s_mat, w, u, vt);
        cv::transpose(vt, v);
        cv::transpose(u, ut);
        //4b. Setup for rotation mat and translation vector
        cv::Mat m_E = cv::Mat::eye(3, 3, CV_32F);
        cv::Mat m_v_ut = v * ut;
        double detVal = cv::determinant(m_v_ut);
        m_E.at<float>(2,2) = detVal;
        cv::Mat rotMat;
        rotMat = v * (m_E * ut);
        cv::Mat trVec;
        trVec = q_centMat - (rotMat*p_centMat);

        //for printouts
        m_matcherCounter++;
        if(m_matcherCounter % 25 == 0)
        {
            m_matcherCounter = 0;
            std::cout << "rotMat:\n" << rotMat << std::endl;
            std::cout << "translation:\n" << trVec << std::endl;
            // std::cout << "u \n" << u << std::endl;
            // std::cout << "ut \n" << ut << std::endl;
            // std::cout << "w \n" << w << std::endl;
            // std::cout << "v \n" << v << std::endl;
            // std::cout << "vt \n" << vt << std::endl;
            
        }
        
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