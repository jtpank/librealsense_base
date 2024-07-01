// include OpenCV header file
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <memory>
using namespace std;
using namespace cv;

class FrameProcessor
{
    private:
        cv::Ptr<cv::ORB> pOrb;
        float depthScale;
    public:
        FrameProcessor() {
            std::cout << "Constructing frame processor object." << std::endl;
            pOrb = cv::ORB::create(); 
        };

        void wrapGoodFeatures(cv::Mat &inputFrame, cv::Mat &depthFrame) {
            const auto goodFeatsWindow = "Good Features Image";
            cv::namedWindow(goodFeatsWindow, cv::WINDOW_AUTOSIZE);

            cv::Mat workFrame, grayFrame;
            vector<Point2f> corners[1];
            inputFrame.copyTo(workFrame);
            cv::cvtColor(workFrame, grayFrame, COLOR_BGR2GRAY);

            //Parameters should move elswhere
            int max_count = 100; 
            double quality_level = 0.01; 
            double min_distance = 3;
            cv::goodFeaturesToTrack(grayFrame, corners[0], max_count, quality_level, min_distance);
            std::cout << "Corners length: " << corners.size() << std::endl;
            // for(auto &corner : corners)
            // {
                
            // }
            while(cv::waitKey(1))
            {
                cv::imshow(goodFeatsWindow, frame);
            }

            return;
        };

};


int main()
{
    cv::Mat inputFrame = cv::imread("./test-image.png", IMREAD_COLOR);
    cv::Mat depthFrame;
    std::unique_ptr<FrameProcessor> fp_ptr = std::make_unique<FrameProcessor>();
    fp_ptr->wrapGoodFeatures(inputFrame, depthFrame);

    bool doPipeline = false;
    if(doPipeline)
    {
        //Contruct a pipeline which abstracts the device
        rs2::pipeline pipe;

        //Create a configuration for configuring the pipeline with a non default profile
        rs2::config cfg;

        //Add desired streams to configuration
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
        //cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
        //cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        //Instruct pipeline to start streaming with the requested configuration
        pipe.start(cfg);
        const auto window_name = "Display Image";
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

        while (cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
        {
            // Camera warmup - dropping several first frames to let auto-exposure stabilize
            rs2::frameset frames;
            try {
                frames = pipe.wait_for_frames();
            } catch (const rs2::error & e) {
                std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n" << e.what() << std::endl;
                continue;
            }
            //Get each frame
            rs2::frame ir_frame = frames.get_infrared_frame(1);
            rs2::frame ir_frame_2 = frames.get_infrared_frame(2);
            //rs2::frame depth_frame = frames.get_depth_frame();
            if (!ir_frame) 
            {
                continue;
            }
            // Creating OpenCV matrix from IR image
            Mat ir(Size(640, 480), CV_8UC1, (void*)ir_frame.get_data(), Mat::AUTO_STEP);

            // Apply Histogram Equalization
            equalizeHist( ir, ir );
            applyColorMap(ir, ir, COLORMAP_JET);
            imshow(window_name, ir);
        }
    }
    return 0;
}
