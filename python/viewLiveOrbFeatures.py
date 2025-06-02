import pyrealsense2 as rs
import numpy as np
import cv2

class RotationEstimator:
    def __init__(self):
        self.theta = np.array([0.0,0.0,0.0])
        self.alpha = 0.98
        self.firstGyro = True
        self.firstAccel = True
        self.last_ts_gryo = 0.0
    def process_gyro(self, gyro_data, ts):
        # On the first iteration, use only data from accelerometer to set the camera's initial position
        if self.firstGyro:
            self.firstGyro = False
            self.last_ts_gyro = ts
            return
        gyro_angle = np.array([0.0,0.0,0.0])
        gyro_angle[0] = gyro_data.x
        gyro_angle[1] = gyro_data.y
        gyro_angle[2] = gyro_data.z

        # compute the difference between arrival times of previous and current gyro frames
        dt_gyro = (ts - self.last_ts_gyro) / 1000.0
        # print(f"\tgyro_ts = {gyro_ts}, last = {self.last_ts_gyro}, Δt = {(gyro_ts - self.last_ts_gyro)}")

        self.last_ts_gyro = ts
        # Change in angle equals gyro measures * time passed since last measurement
        gyro_angle = gyro_angle * dt_gyro
        # print(f"\t gyro_data: {gyro_data.x}, {gyro_data.y}, {gyro_data.z}")
        # apply the calculated change of angle to the current angle (theta)
        self.theta = self.theta + np.array([-gyro_angle[2], -gyro_angle[1], gyro_angle[0]])

    def process_accel(self, accel_data):
        accel_angle = np.array([0.0,0.0,0.0])
        # Calculate rotation angle from accelerometer data
        accel_angle[2] = np.arctan2(accel_data.y, accel_data.z)
        accel_angle[0] = np.arctan2(accel_data.x, np.sqrt(accel_data.y ** 2 + accel_data.z ** 2))
        #If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
        if self.firstAccel:
            self.firstAccel = False
            self.theta[2] = accel_angle[2]
            self.theta[0] = accel_angle[0]
            # Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
            self.theta[1] = np.pi
        else:
            #Apply Complementary Filter:
            #    - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
            #      that are steady over time, is used to cancel out drift.
            #    - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations 
            self.theta[0] = self.theta[0] * self.alpha + accel_angle[0] * (1 - self.alpha)
            self.theta[2] = self.theta[2] * self.alpha + accel_angle[2] * (1 - self.alpha)
    def get_theta(self):
        return self.theta

HEIGHT = 480
WIDTH = 640
if __name__ == "__main__":
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.accel)
    config.enable_stream(rs.stream.gyro)
    profile = pipeline.start(config)
    align_to = rs.stream.color  # or rs.stream.depth, depending on your target
    align = rs.align(align_to)
    orb = cv2.ORB_create()
    rot = RotationEstimator()
    device = profile.get_device()
    # for sensor in device.query_sensors():
    #     print(f"\nSensor: {sensor.get_info(rs.camera_info.name)}")
    #     for sp in sensor.get_stream_profiles():
    #         fmt = sp.format()
    #         stream_type = sp.stream_type()
    #         print(f"  Stream: {stream_type}, Format: {fmt}, FPS: {sp.fps()}")
    seen_first_frame = False
    prev_gray_image = np.empty((WIDTH, HEIGHT), dtype=np.uint8)

    feature_params = dict( maxCorners = 200,
                       qualityLevel = 0.3,
                       minDistance = 5,
                       blockSize = 7 )
    # Create some random colors
    color = np.random.randint(0, 255, (100, 3))
    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (15, 15),
                    maxLevel = 2,
                    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    mask = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    p0 = None
    MIN_TRACKED_POINTS = 20  # NEW: threshold for refreshing features
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        accel_frame = aligned_frames.first(rs.stream.accel, rs.format.motion_xyz32f)
        a_gf = aligned_frames.first(rs.stream.gyro, rs.format.motion_xyz32f)
        if not color_frame or not accel_frame or not a_gf:
            continue
        color_image = np.asanyarray(color_frame.get_data())
        curr_gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        if not seen_first_frame:
            prev_gray_image = curr_gray_image
            seen_first_frame = True
            p0 = cv2.goodFeaturesToTrack(prev_gray_image, mask = None, **feature_params)
            continue
        color_colormap_dim = color_image.shape
        # kp, des = orb.detectAndCompute(color_image, None)
        # kp_image = cv2.drawKeypoints(color_image, kp, None, color=(0,255,0), flags=0)

        # goodFeatures = cv2.goodFeaturesToTrack(np.mean(color_image, axis=2).astype(np.uint8),mask = None, **feature_params)
        
        if p0 is None or len(p0) < MIN_TRACKED_POINTS:
            p0 = cv2.goodFeaturesToTrack(curr_gray_image, mask=None, **feature_params)
            prev_gray_image = curr_gray_image.copy()
            mask = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)  # NEW: reset mask to clear old lines
            continue

        # calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(prev_gray_image, curr_gray_image, p0, None, **lk_params)
        if p1 is None or st is None or st.sum() == 0:
            goodFeatures = cv2.goodFeaturesToTrack(np.mean(color_image, axis=2).astype(np.uint8),mask = None, **feature_params)
            continue
        # if goodFeatures is None:
        #     continue
        # for pt in goodFeatures:
        #     pt_tuple = (pt[0][0],pt[0][1])
        #     cv2.circle(color_image,pt_tuple,radius=3,color=(0,255,0))
        # Select good points
        good_new = p1[st==1]
        good_old = p0[st==1]

        # draw the tracks
        frame = color_image.copy()
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
            frame = cv2.circle(frame, (int(a), int(b)), 5, color[i].tolist(), -1)
        
        track_img = cv2.add(frame, mask)
        combined_img = np.hstack((color_image, track_img))

        # IMU computation
        gyro_ts = a_gf.get_timestamp()
        # kp_image = cv2.drawKeypoints(color_image, goodFeatures, None, color=(0,255,0), flags=0)
        if accel_frame:
            av = accel_frame.as_motion_frame().get_motion_data()
            rot.process_accel(av)
        if a_gf:
            gv = a_gf.as_motion_frame().get_motion_data()
            rot.process_gyro(gv, gyro_ts)
        outputTheta = (rot.get_theta())* 180.0 / np.pi
        print(f"Angles:  Pitch={outputTheta[0]:.1f}°, Yaw={outputTheta[1]:.1f}°, Roll={outputTheta[2]:.1f}°")

        cv2.namedWindow('combinedView', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('combinedView', combined_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # update
        prev_gray_image = curr_gray_image.copy()
        p0 = good_new.reshape(-1, 1, 2)

    cv2.destroyAllWindows()
    pipeline.stop()
    
