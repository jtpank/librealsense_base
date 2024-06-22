// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);
    const auto window_name = "Display Image";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    while (cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
    {
        // Camera warmup - dropping several first frames to let auto-exposure stabilize
        rs2::frameset frames;

        //Get each frame
        rs2::frame ir_frame = frames.first(RS2_STREAM_INFRARED);
        rs2::frame depth_frame = frames.get_depth_frame();
        if (!ir_frame || !depth_frame) 
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

    return 0;
}