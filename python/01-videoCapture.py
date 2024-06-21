import pyrealsense2 as rs
import numpy as np
import cv2
from FrameProcessor import FrameProcessor
# d435i depth camera properties
# baseline 50mm
# f number f/2.0
# focal length 1.93mm
# https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Intel-RealSense-D400-Series-Datasheet.pdf#page=32&zoom=100,184,605
# depth = baseline*focal_length / disparity
def process_depth_values(frame):
    pass
def convert16BitDepthToTwo8Bit3Channel(frame):
    aFrame = np.zeros(frame.shape, dtype=int)
    bFrame = np.zeros(frame.shape, dtype=int)
    cFrame = np.zeros(frame.shape, dtype=int)

    shape = frame.shape
    w = shape[0]
    h = shape[1]
    for c in w:
        for r in h:
            hiByte, loByte= divmod(frame[w, h], 256)
            aFrame[w, h] = hiByte
            bFrame[w, h] = loByte
    outputFrame = np.dstack((aFrame, bFrame, cFrame))
    return outputFrame

if __name__ == "__main__":
    pipeline = rs.pipeline()
    config = rs.config()
    # config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 30)
    # config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth scale is: ", depth_scale)
    clip_dist_m = 1
    clip_dist = clip_dist_m / depth_scale
    align_to = rs.stream.color
    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align = rs.align(align_to)

    colorVideoFile = "./colorVideo.avi"
    frameRate = 30
    dim = (640,480)
    #fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    #outputColorWriter = cv2.VideoWriter(colorVideoFile,fourcc,frameRate,dim)
    color_image_array = np.array([[]])
    depth_image_array = np.array([[]])
    npColorPath = "./colorFrames.npy"
    npDepthPath = "./depthFrames.npy"
    fpr = FrameProcessor(depth_scale=depth_scale)
    while True:
        frames = pipeline.wait_for_frames()
        #align depth frame to color frame
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not color_frame or not aligned_depth_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_colormap_dim = color_image.shape
        #grey_color = 153
        #outputCustomFrame = convert16BitDepthToTwo8Bit3Channel(depth_image)
        #depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        #bg_removed = np.where((depth_image_3d > clip_dist) | (depth_image_3d <= 0), grey_color, color_image)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        colorFrame, depthFrame, _, _ = fpr.wrap_goodFeats(color_image, depth_colormap, depth_image)
        #note that shape is not the same as dim (480,640, 3) vs (480, 640)
        both_images = np.hstack((colorFrame, depthFrame))
        #outputColorWriter.write(color_image.astype('uint8'))
        #outputDepthWriter.write(outputCustomFrame.astype('uint8'))
        #print(depth_image.shape)
        cv2.namedWindow('test show vid', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('test show vid', both_images)
        if cv2.waitKey(1) == ord('q'):
            break
    #outputWriter.release()
    cv2.destroyAllWindows()
    pipeline.stop()
