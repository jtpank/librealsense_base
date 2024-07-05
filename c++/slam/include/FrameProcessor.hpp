#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/types.hpp>
#include <cassert>
#include <librealsense2/rs.hpp> 
#include "FrameBuffer.hpp"
#include <iostream>
#include <thread>
#include <vector>

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
        bool m_hasFirstFrame;
        std::deque<std::vector<cv::KeyPoint>> m_kps;
        std::deque<cv::Mat> m_des;
    public:
        FrameProcessor(unsigned int poolSize);
        void frameConsumer(int threadId);
        void processFrameset(rs2::frameset& frameSet);
        void set_depthScale(float depthScale);
        void wrapGoodFeatures(cv::Mat &inputFrame, cv::Mat &outputFrame);
        void orbDetectAndCompute(cv::Mat &inputFrame, cv::Mat &outputFrame);
        void bfMatchFrames();
        void test_wrapGoodFeatures();
};