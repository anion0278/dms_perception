import pyrealsense2 as rs

   
def transform_px_to_pt(intr,px,depth):
    return rs.rs2_deproject_pixel_to_point(intr,px,depth)

def transform_pt_to_base(extrpt):
    return rs.rs2_transform_point_to_point(extr,pt)