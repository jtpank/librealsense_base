#include "FrameProcessor.hpp"

FrameProcessor::FrameProcessor(unsigned int poolSize) 
{   
    std::cout << "Constructing frame processor object: " << std::endl;
    try {
        m_pOrb = cv::ORB::create();
    }
    catch(...) {
        std::cout << "Error: with cv::ORB::create() in constructor." << std::endl;
    }
    std::cout << "Constructing FrameBuffer: " << std::endl;
    try {
        m_frameBuffer(poolSize);
    }
    catch(...) {
        std::cout << "Error: constructing FrameBuffer." << std::endl;
    }

    std::cout << "Setting up thread pool: " << std::endl;
    m_poolSize = poolSize;
    for(auto i = 0; i < poolSize; ++i)
    {
        //this leads to terminate called without an active exception
        // Aborted (core dumped)
        try {
             m_pool.emplace_back(std::thread([=](){ this->processFrame(i); }));
        } catch (const std::exception& e) {
            std::cerr << "Exception during thread creation: " << e.what() << std::endl;
            throw;
        }
    }
}

void FrameProcessor::processFrame(int threadId)
{
    std::cout << "thread number: " << threadId << std::endl;
}

void FrameProcessor::set_depthScale(float depthScale) 
{
    m_depthScale = depthScale;
}
void FrameProcessor::wrapGoodFeatures(cv::Mat &inputFrame, cv::Mat &depthFrame, cv::Mat &outputFrame) 
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
    cv::Mat depthFrame, outputFrame;

    wrapGoodFeatures(inputFrame, depthFrame, outputFrame);
    const auto goodFeatsWindow = "Good Features Image"; 
    cv::namedWindow(goodFeatsWindow, cv::WINDOW_AUTOSIZE);

    while(cv::waitKey(1))
    {
        cv::imshow(goodFeatsWindow, outputFrame);
    }

}