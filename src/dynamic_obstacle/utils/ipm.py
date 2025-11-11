import numpy as np
import cv2

def homography(src_pts, dst_pts):
    src = np.array(src_pts, dtype=np.float32)
    dst = np.array(dst_pts, dtype=np.float32)
    H, _ = cv2.findHomography(src, dst, method=0)
    return H

def warp_to_ground(img, H, out_size=(400, 300), scale=400):
    # dst coords are in meters; scale to pixels for visualization
    warped = cv2.warpPerspective(img, H, out_size)
    return warped

def polygon_mask(shape, pts):
    mask = np.zeros(shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [np.array(pts, dtype=np.int32)], 255)
    return mask
