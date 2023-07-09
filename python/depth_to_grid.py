import pyrealsense2 as rs
import numpy as np
import cv2


class OccupancyGrid:
    def __init__(self, resolution, side_length, default_value=0):
        #initializes a square occupancy grid depending on the side_length and resolution
        self.__width = side_length // resolution
        self.__height = side_length // resolution
        self.__default_value = default_value
        self.__grid = np.full((self.__width, self.__height), self.__default_value)

    def get_value(self, x, y):
        return self.__grid[y][x]

    def set_value(self, x, y, value):
        self.__grid[y][x] = value

    def clear_grid(self, value=0):
        self.__grid = np.full((self.__width, self.__height), self.__default_value)

    def print_grid(self):
        for row in self.__grid:
            print(' '.join(map(str, row)))

    #log odds update algorithm
    # input row is the data from the camera
    def update_grid(self, input_row):
        for cell in self.__grid:
            if cell in input_row:
                # log_odds_i = prev_log_odds_i + inv_sensor_model(cell, state, observation) - log_odds_initial
                pass
            else:
                # log_odds_i = prev_log_odds_i
                pass
        return
    
    #implement Bresenhems line algorithm for 2 points 
    # pt1 = (x1, y1) is camera origin, pt2 = (x2, y2) is point with non-zero depth reading
    def draw_line(self, x1, y1, x2, y2):
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while x1 != x2 or y1 != y2:
            self.set_value(x1, y1, 1)  # Set the value of the grid cell to 1
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy

        self.set_value(x2, y2, 1)  # Set the value of the end point to 1


def main():
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
            row = depth_frame.get_height() // 2

            depth_middle_row = []
            start_col = depth_frame.get_width() // 4
            end_col = depth_frame.get_width() // 2 + start_col
            for col in range(start_col, end_col):
                depth_middle_row.append(depth_frame.get_distance(row, col))

            depth_middle_row = np.array(depth_middle_row)

    finally:
        # Stop streaming
        pipeline.stop()

def test_grid():
    grid = OccupancyGrid(1, 10)  # Create a 10x10 grid with resolution 1
    grid.draw_line(1, 1, 9, 4)
    grid.print_grid()
    grid.clear_grid()
    print("\n ****************************************************** \n")
    grid.draw_line(1, 1, 9, 3)
    grid.print_grid()
    grid.clear_grid()
    print("\n ****************************************************** \n")
    grid.draw_line(9, 3, 0, 9)
    grid.print_grid()
    grid.clear_grid()
if __name__ == "__main__":
    test_grid()
    