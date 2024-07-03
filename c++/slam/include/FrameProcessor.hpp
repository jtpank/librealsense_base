#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

class FrameProcessor
{
    private:
        cv::Ptr<cv::ORB> _pOrb;
        float _depthScale;
    public:
        FrameProcessor();
        void set_depthScale(float ds);
        void wrapGoodFeatures(cv::Mat &inputFrame, cv::Mat &depthFrame, cv::Mat &outputFrame);
        void test_wrapGoodFeatures();
};