import pyrealsense2 as rs
import numpy as np
import time
import meshcat
import meshcat.geometry as g

W = 640
H = 480

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)


print("[INFO] start streaming...")
pipeline.start(config)

aligned_stream = rs.align(rs.stream.color) # alignment between color and depth
point_cloud = rs.pointcloud()

vis = meshcat.Visualizer().open()

while True:
    frames = pipeline.wait_for_frames()
    frames = aligned_stream.process(frames)
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame().as_depth_frame()

    points = point_cloud.calculate(depth_frame)
    verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(W * H, 3)
    verts = np.transpose(verts)
    vis.set_object(g.Points(g.PointsGeometry(verts, color=verts), g.PointsMaterial()))
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    # skip empty frames
    if not np.any(depth_image):
        continue
    color_image = np.asanyarray(color_frame.get_data())
    

# Stop streaming
pipeline.stop()