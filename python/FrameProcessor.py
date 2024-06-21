import numpy as np
import cv2


class FrameProcessor():
    def __init__(self, depth_scale):
        self.orb = cv2.ORB_create()
        self.depth_scale = depth_scale
#finds features and draws on the depth colormap if the distance is
# within a range of 0.1 to < 1 meters    
    def wrap_goodFeats(self, frame, dframe, raw_dframe):
        out_frame = frame.copy()
        out_dframe = dframe.copy()
        grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(grayFrame, maxCorners=100, qualityLevel=0.01, minDistance=3)
        for idx, corner in enumerate(corners):
            pt = (int(corner[0][0]), int(corner[0][1]))
            cv2.circle(out_frame, pt, radius=2, color=(0,255,0))
            sz = 0.3
            cv2.putText(out_frame, str(idx), pt,cv2.FONT_HERSHEY_SIMPLEX, sz, color=(0,0,255))
            cv2.circle(out_dframe, pt, radius=2, color=(0,255,0))
            #round as well
            dist = raw_dframe[pt[1], pt[0]] * self.depth_scale
            if dist <= 1 and dist > 0.1:
                cv2.putText(out_dframe, str(round(dist,2)), pt,cv2.FONT_HERSHEY_SIMPLEX, sz, color=(0,0,255))

        keypoints = [cv2.KeyPoint(x=p[0][0], y=p[0][1], size=20) for p in corners]
        kps, des = self.orb.compute(frame, keypoints)
        arrKp = np.array([(kp.pt[0], kp.pt[1]) for kp in kps])
        return out_frame, out_dframe, arrKp, des
    


def main():
    pass

if __name__=="__main__":
    main()
