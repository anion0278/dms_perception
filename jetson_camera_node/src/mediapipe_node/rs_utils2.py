import pyrealsense2 as rs

class rs_utils2:
   
    def transform_px_to_pt(self,intr,px,depth):
        return rs.rs2_deproject_pixel_to_point(intr,px,depth)

    def transform_pt_to_base(self,extrpt):
        return rs.rs2_transform_point_to_point(extr,pt)