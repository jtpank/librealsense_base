#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/types.hpp>
#include <cassert>
#include <librealsense2/rs.hpp> 
#include "FrameBuffer.hpp"
#include "float3.hpp"
#include "RotationEstimator.hpp"
#include <iostream>
#include <thread>
#include <vector>
#include <cstdio>

//TODO: must contain the consumer threads

class FrameProcessor
{
    private:
        cv::Ptr<cv::ORB> m_pOrb;
        cv::Ptr<cv::BFMatcher> m_bfMatcher;
        float m_depthScale;
        unsigned int m_poolSize;
        std::vector<std::thread> m_pool;
        FrameBuffer<rs2::frameset> m_frameBuffer;

        rs2::pointcloud m_pc;
        rs2::points m_points;

        FrameBuffer<rs2::frame> m_colorFrameBuffer;
        FrameBuffer<rs2::frame> m_depthFrameBuffer;
        FrameBuffer<std::vector<rs2::frame>> m_imuFrameBuffer;
        int m_colorCounter;
        int m_depthCounter;
        int m_imuCounter;

        bool m_hasFirstFrame;
        std::deque<std::vector<cv::KeyPoint>> m_kps;
        std::deque<const rs2::vertex*> m_vertices;
        std::deque<cv::Mat> m_des;
        bool m_shutdownThreads;

        RotationEstimator algo;
    public:
        FrameProcessor(unsigned int poolSize);
        void frameConsumer(int threadId, FrameBufferType ftype);
        void framesetConsumer(int threadId);

        void processFrameset(rs2::frameset& frameSet);
        void processFramesToIndividualBuffers(rs2::frameset& frameSet);
        void consumeColorFrame(int threadId);
        void consumeDepthFrame(int threadId);
        void consumeImuFrame(int threadId);

        void set_depthScale(float depthScale);
        void wrapGoodFeatures(cv::Mat &inputFrame, cv::Mat &outputFrame);
        void orbDetectAndCompute(cv::Mat &inputFrame, cv::Mat &outputFrame);
        void grabVertices(rs2::depth_frame &depth_frame, rs2::points &points, rs2::pointcloud &pc);
        void frameMatcher();
        void poseFromHomography(cv::Mat &H);
        void test_wrapGoodFeatures();
        void joinAllThreads();
};