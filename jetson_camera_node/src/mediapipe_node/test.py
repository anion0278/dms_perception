# tried to use BUILD from https://github.com/JetsonHacksNano/installLibrealsense/blob/master/buildLibrealsense.sh
#sys.path.append("/usr/local/lib/python3.6") # required, otherwise can be solved by https://github.com/IntelRealSense/librealsense/issues/6820#issuecomment-660167748
import pyrealsense2 as rs
p = rs.pipeline() 
pipeline_config = rs.config()
pipeline_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = p.start(pipeline_config)
fs = p.wait_for_frames()
df = fs.get_depth_frame() 

# from ROS message https://medium.com/@yasuhirachiba/converting-2d-image-coordinates-to-3d-coordinates-using-ros-intel-realsense-d435-kinect-88621e8e733a

depth_pixel = [410, 200]
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().intrinsics
#m_intrin = rs2::video_stream_profile(m_depth_frame.get_profile()).get_intrinsics();
depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_scale)
#color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
#color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)
#point = rs.rs2_deproject_pixel_to_point(intr,[20, 20],np.asanyarray(df.get_data()))
#https://github.com/IntelRealSense/librealsense/issues/1231#issuecomment-368421888


x= rs.extrinsics()
print(x)
print(type(x))
pass



		