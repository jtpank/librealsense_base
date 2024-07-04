#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "FrameBuffer.hpp"
#include <iostream>
#include <thread>
#include <vector>

//TODO: must contain the consumer threads

class FrameProcessor
{
    private:
        cv::Ptr<cv::ORB> m_pOrb;
        float m_depthScale;
        unsigned int m_poolSize;
        std::vector<std::thread> m_pool;
        FrameBuffer<int> m_frameBuffer;

    public:
        FrameProcessor(unsigned int poolSize);
        void processFrame(int threadId);
        void set_depthScale(float depthScale);
        void wrapGoodFeatures(cv::Mat &inputFrame, cv::Mat &depthFrame, cv::Mat &outputFrame);
        void test_wrapGoodFeatures();
};