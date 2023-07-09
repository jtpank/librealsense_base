import pyrealsense2 as rs
import numpy as np
import cv2


if __name__ == "__main__":
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            
            # Convert images to numpy arrays
            depth_image_data = np.asanyarray(depth_frame.get_data())
            depth_image_data_dim = depth_image_data.shape

            #take the middle row of pixels
            row = depth_frame.get_height() / 2

            depth_middle_row = []
            start_col = depth_frame.get_width() / 4
            end_col = depth_frame.get_width() / 2 + start_col
            for col in range(start_col, end_col):
                depth_middle_row.append(depth_frame.get_distance(row, col))

            depth_middle_row = np.array(depth_middle_row)
            print("_______________________________________________")
            print(depth_image_data_dim)
            print(depth_middle_row)
    finally:
        # Stop streaming
        pipeline.stop()