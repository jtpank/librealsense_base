import pyrealsense2 as rs
import numpy as np
import cv2


if __name__ == "__main__":
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    videoFile = "./testVideo.avi"
    frameRate = 30
    dim = (640,480)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    outputWriter = cv2.VideoWriter(videoFile,fourcc,frameRate,dim)
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())
        color_colormap_dim = color_image.shape
        #note that color_colormap_dim is not the same as dim (480,640, 3) vs (480, 64)
        outputWriter.write(color_image.astype('uint8'))
        cv2.namedWindow('test show vid', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('test show vid', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    outputWriter.release()
    cv2.destroyAllWindows()
    pipeline.stop()
        
    print("hi")
