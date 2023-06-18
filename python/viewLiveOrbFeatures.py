import pyrealsense2 as rs
import numpy as np
import cv2


if __name__ == "__main__":
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    orb = cv2.ORB_create()
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())
        color_colormap_dim = color_image.shape
        # kp, des = orb.detectAndCompute(color_image, None)
        # kp_image = cv2.drawKeypoints(color_image, kp, None, color=(0,255,0), flags=0)
        goodFeatures = cv2.goodFeaturesToTrack(np.mean(color_image, axis=2).astype(np.uint8),
                                               maxCorners=3000, 
                                               qualityLevel=0.01,
                                               minDistance=3)
        for pt in goodFeatures:
            pt_tuple = (pt[0][0],pt[0][1])
            cv2.circle(color_image,pt_tuple,radius=3,color=(0,255,0))
        # kp_image = cv2.drawKeypoints(color_image, goodFeatures, None, color=(0,255,0), flags=0)
        cv2.namedWindow('kpImage', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('kpImage', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    pipeline.stop()
    
