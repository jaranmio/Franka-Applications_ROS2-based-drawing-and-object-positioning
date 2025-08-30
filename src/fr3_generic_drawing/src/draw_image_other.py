import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import math
import random
import time
import os

# Import modules from the image processing code
from LDR import *
from tone import *
from genStroke_origin import *
from drawpatch import rotate
from tools import *
from ETF.edge_tangent_flow import *
from deblue import deblue
from quicksort import *

# Import ROS2 MoveIt2 interfaces
import rclpy
from moveit2 import MoveIt2
from geometry_msgs.msg import PoseStamped

# --------------- Parameters ---------------

# File paths
input_path = './input/cat_up.png'   # Path to input image
output_path = './output'           # Base output directory for saving results

# Image processing parameters
np.random.seed(1)
n = 6                # Quantization order (number of tone levels)
period = 4           # Line patch height (pixels) and base line spacing
direction = 10       # Number of stroke directions (orientations from -90 to +90 degrees)
deepen = 1           # Exponent for edge darkening (1 = no change)
transTone = False    # Whether to apply tone transfer (Tone8) to input image
kernel_radius = 3    # ETF filter kernel radius
iter_time = 15       # ETF iteration count
background_dir = 45  # Background direction threshold for ETF 
CLAHE = True         # Apply CLAHE histogram equalization to each rotated image
edge_CLAHE = True    # Apply CLAHE to final edge mask
draw_new = True      # If True, generate strokes anew
random_order = False # If True, shuffle strokes randomly
ETF_order = True     # If True, sort strokes by importance (using ETF guidance)
process_visible = False  # If True, show drawing process in a window (not recommended in headless mode)

# Robot and drawing parameters
planning_group_name = "panda_arm"    # MoveIt planning group for the FR3 arm (adjust as needed)
base_frame = "panda_link0"           # Base frame for robot poses (e.g., robot base link or world frame)
# Physical drawing area size (meters) corresponding to the image dimensions:
drawing_width = 0.30   # Width of drawing in meters (x-direction range)
drawing_height = 0.30  # Height of drawing in meters (y-direction range)
# If maintaining aspect ratio, ensure drawing_width/drawing_height = original_width/original_height.
# Otherwise, adjust one of them or accept slight distortion.

plane_height = 0.0     # Z-height (in meters) of the drawing surface (e.g., table or whiteboard surface) in the robot's base_frame
lift_distance = 0.05   # Pen lift distance in meters (height above surface when moving between strokes)
# Orientation for the end-effector such that pen is perpendicular to drawing surface, pointing downwards:
# (The example here is a quaternion pointing the end-effector down; adjust for your robot's orientation as needed)
target_orientation = [0.0, 0.7071, 0.0, 0.7071]  # [x, y, z, w] quaternion (example rotates end-effector 90° around X-axis)

# --------------- Prepare output directories ---------------
file_name = os.path.basename(input_path).split('.')[0]
output_path = os.path.join(output_path, file_name)
if not os.path.exists(output_path):
    os.makedirs(output_path)
    os.makedirs(os.path.join(output_path, "mask"))
    os.makedirs(os.path.join(output_path, "process"))

# --------------- Step 1: Edge Tangent Flow (ETF) for stroke directions ---------------
time_start = time.time()
ETF_filter = ETF(
    input_path=input_path,
    output_path=os.path.join(output_path, "mask"),
    dir_num=direction,
    kernel_radius=kernel_radius,
    iter_time=iter_time,
    background_dir=background_dir
)
ETF_filter.forward()
print("ETF filtering done.")

# --------------- Step 2: Load input image and preprocess ---------------
input_img = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)
(h0, w0) = input_img.shape
cv2.imwrite(os.path.join(output_path, "input_gray.jpg"), input_img)
# Optionally resize input image here if needed (code commented out in original)
if transTone:
    input_img = transferTone(input_img)

# --------------- Step 3: Generate strokes from the image ---------------
stroke_sequence = []  # list to hold stroke dictionaries
# Dictionary to store rotation parameters for each angle (to avoid recomputation)
angle_params = {}

if draw_new:
    gen_start = time.time()
    for dirs in range(direction):
        angle = -90 + dirs * 180.0 / direction  # orientation of strokes in degrees for this loop
        print(f"Processing angle {angle:.1f}° (direction {dirs})...")
        # Rotate the input image by -angle to align strokes horizontally
        rotated_img, _ = rotate(input_img, -angle)
        # Apply CLAHE histogram equalization if enabled
        if CLAHE:
            rotated_img = HistogramEqualization(rotated_img)
        print("Histogram equalization done.")
        # Compute image gradients on the padded rotated image
        img_pad = cv2.copyMakeBorder(rotated_img, 2*period, 2*period, 2*period, 2*period, cv2.BORDER_REPLICATE)
        img_normal = cv2.normalize(img_pad.astype("float32"), None, 0.0, 1.0, cv2.NORM_MINMAX)
        x_der = cv2.Sobel(img_normal, cv2.CV_32FC1, 1, 0, ksize=5)
        y_der = cv2.Sobel(img_normal, cv2.CV_32FC1, 0, 1, ksize=5)
        x_der = torch.from_numpy(x_der) + 1e-12
        y_der = torch.from_numpy(y_der) + 1e-12
        gradient_magnitude = torch.sqrt(x_der**2 + y_der**2)
        gradient_norm = gradient_magnitude / gradient_magnitude.max()
        # Quantize the rotated image into n tone levels
        ldr = LDR(rotated_img, n)
        cv2.imwrite(os.path.join(output_path, "Quantization.png"), ldr)
        # Generate binary masks for each tone level and cumulative tone image
        LDR_single_add(ldr, n, output_path)
        print("Quantization and mask generation done.")
        (h_rot, w_rot) = ldr.shape  # dimensions of rotated image
        # Prepare Gaussian noise canvas (for stroke simulation, not strictly needed for stroke list)
        canvas = Gassian((h_rot + 4*period, w_rot + 4*period), mean=250, var=3)
        # Compute rotation parameters (nW, nH, cos, sin) for this angle (if not already stored)
        if angle not in angle_params:
            theta = math.radians(angle)
            # Since rotated by -angle, the bounding box size for rotated image:
            nW = int(abs(w0 * math.cos(theta)) + abs(h0 * math.sin(theta)))
            nH = int(abs(w0 * math.sin(theta)) + abs(h0 * math.cos(theta)))
            cosA = math.cos(math.radians(angle))   # cos of original stroke angle (note: same as cos(-angle) since cos is even)
            sinA = math.sin(math.radians(angle))   # sin of original stroke angle
            angle_params[angle] = (nW, nH, cosA, sinA)
        # Iterate through each tone layer j for this direction
        for j in range(n):
            stroke_base = {'angle': angle, 'grayscale': j * 256 / n, 'row': None, 'begin': None, 'end': None, 'importance': None}
            # Load the mask for tone j and the directional mask for this orientation
            mask = cv2.imread(os.path.join(output_path, 'mask', f'mask{j}.png'), cv2.IMREAD_GRAYSCALE) / 255.0
            dir_mask = cv2.imread(os.path.join(output_path, 'mask', f'dir_mask{dirs}.png'), cv2.IMREAD_GRAYSCALE)
            # Rotate the directional mask to align with this orientation
            dir_mask, _ = rotate(dir_mask, -angle, pad_color=0)
            # Binarize the rotated directional mask
            dir_mask[dir_mask < 128] = 0
            dir_mask[dir_mask >= 128] = 1
            # Create a randomized distribution of line spacings around 'period'
            distance = Gassian((1, int(h_rot/period) + 4), mean=period, var=1)
            distance = np.uint8(np.round(np.clip(distance, period*0.8, period*1.25)))
            raw_y = -int(period/2)  # start with a half-period offset (can be negative to start above the image)
            # Sweep through the image rows for this tone, stepping by random distances
            for step_dist in np.squeeze(distance).tolist():
                if raw_y < h_rot:
                    y = raw_y + 2*period  # offset by padding in canvas coordinates
                    raw_y += step_dist   # move to the next stroke row position
                    # Determine continuous segments (intervals) on this row where mask and dir_mask overlap (i.e., stroke should be drawn)
                    row_mask = mask[y - 2*period] * dir_mask[y - 2*period]  # this corresponds to the rotated image row
                    intervals = get_start_end(row_mask)
                    for interval in intervals:
                        begin = int(interval[0])
                        end = int(interval[1])
                        # Expand the interval by 2*period on each side (to capture stroke width)
                        begin_ext = begin - 2*period
                        end_ext = end + 2*period
                        # Clip to image bounds (in case extended beyond edges)
                        if begin_ext < -2*period:
                            begin_ext = -2*period
                        if end_ext > w_rot - 1 + 2*period:
                            end_ext = w_rot - 1 + 2*period
                        # Compute stroke importance as (255 - tone) * sum of gradient norm in that region
                        imp_region = gradient_norm[y : y+period, (begin + 2*period) : (end + 2*period)]
                        importance_val = (255 - stroke_base['grayscale']) * torch.sum(imp_region).item()
                        # Store the stroke information
                        stroke_info = stroke_base.copy()
                        stroke_info['row'] = y - int(period/2)   # top of stroke patch in canvas coordinates
                        stroke_info['begin'] = begin_ext
                        stroke_info['end'] = end_ext
                        stroke_info['importance'] = importance_val
                        stroke_sequence.append(stroke_info)
                        # (Drawing the stroke on canvas for visualization is skipped here)
    gen_end = time.time()
    print(f"Stroke extraction complete. Total strokes found: {len(stroke_sequence)} in {gen_end - gen_start:.2f} seconds.")

    # Shuffle or sort strokes as specified by settings
    if random_order:
        random.shuffle(stroke_sequence)
    if ETF_order:
        # The quickSort function sorts in-place based on 'importance' key (most likely descending order)
        random.shuffle(stroke_sequence)  # mix first, then sort for a more varied distribution
        quickSort(stroke_sequence, 0, len(stroke_sequence) - 1)
else:
    print("draw_new is False, skipping stroke generation (expects stroke_sequence from previous run).")

# --------------- Step 4: Generate final pencil drawing images (optional) ---------------
# We now simulate drawing all strokes onto a blank canvas to produce a final pencil-style image (grayscale),
# and then merge with edge mask for darker outlines.

# Initialize blank canvas for final drawing
result_canvas = np.uint8(np.ones((h0, w0)) * 255)  # start with white canvas
# Prepare individual direction canvases (for rotating into orientation)
canvases = []
for dirs in range(direction):
    angle = -90 + dirs * 180.0 / direction
    canvas, _ = rotate(result_canvas, -angle)
    canvas = np.pad(canvas, pad_width=2*period, mode='constant', constant_values=255)
    canvases.append(canvas)

# Draw each stroke onto the canvases (simulate the pencil drawing result)
step_count = 0
for stroke in stroke_sequence:
    angle = stroke['angle']
    dirs_index = int(round((angle + 90) * direction / 180.0))  # map angle back to index (approx)
    if dirs_index < 0: dirs_index = 0
    if dirs_index >= len(canvases): dirs_index = len(canvases) - 1
    canvas = canvases[dirs_index]
    grayscale = stroke['grayscale']
    # Generate a stroke patch (newline) of length equal to stroke segment length
    length = stroke['end'] - stroke['begin']
    distribution = ChooseDistribution(period=period, Grayscale=grayscale)
    newline_patch = Getline(distribution=distribution, length=length)
    # Determine placement on the rotated canvas
    r = stroke['row']
    c_start = 2*period + stroke['begin']
    c_end = 2*period + stroke['end']
    # Blend the stroke patch onto the canvas using minimum (darker value wins)
    if length < 1000 or stroke['begin'] == -2*period or stroke['end'] == w0 - 1 + 2*period:
        # If stroke is small or touches boundary, use partial patch width
        temp_region = canvas[r : r + 2*period, c_start : c_end]
        overlap = newline_patch[:, :temp_region.shape[1]]
        canvas[r : r + 2*period, c_start : c_end] = np.minimum(temp_region, overlap)
    else:
        # For very long strokes, use the full patch (this branch may not be needed often)
        temp_region = canvas[r : r + 2*period, c_start - 2*period : c_end + 2*period]
        canvas[r : r + 2*period, c_start - 2*period : c_end + 2*period] = np.minimum(temp_region, newline_patch)
    # Rotate the updated canvas patch back to original orientation and merge with result_canvas
    rotated_back, _ = rotate(canvas[2*period:-2*period, 2*period:-2*period], angle)
    (H, W) = rotated_back.shape
    # Center-crop to original size in case rotation produced a larger image
    cropped = rotated_back[int((H - h0) / 2) : int((H - h0) / 2) + h0,
                           int((W - w0) / 2) : int((W - w0) / 2) + w0]
    result_canvas = np.minimum(result_canvas, cropped)
    step_count += 1
    # Optionally visualize progress (disabled by default for headless execution)
    # if process_visible:
    #     cv2.imshow('step', result_canvas)
    #     cv2.waitKey(1)

# Save the simulated pencil drawing (before adding edges)
cv2.imwrite(os.path.join(output_path, "draw.jpg"), result_canvas)

# Generate edge mask for stronger outlines (using genStroke from provided code)
edge_mask = genStroke(input_img, 18)  # 18-direction stroke mask from original image
edge_mask = np.power(edge_mask, deepen)
edge_mask = np.uint8(edge_mask * 255)
if edge_CLAHE:
    edge_mask = HistogramEqualization(edge_mask)
cv2.imwrite(os.path.join(output_path, "edge.jpg"), edge_mask)

# Merge the strokes image and edge mask to produce final pencil drawing
edge_f = np.float32(edge_mask)
res_cross = np.float32(result_canvas)
merged = np.empty_like(res_cross)
# Blend by multiplying edge intensity with stroke intensity (darker lines where edges are)
merged = edge_f * res_cross / 255.0
merged = np.uint8(merged)
cv2.imwrite(os.path.join(output_path, "result.jpg"), merged)

# Remove color cast (deblue if needed, assuming it corrects blue tint in scans - optional)
deblue(merged, output_path)

# Convert result to color by combining with original color image (if desired)
img_rgb_original = cv2.imread(input_path, cv2.IMREAD_COLOR)
cv2.imwrite(os.path.join(output_path, "input.jpg"), img_rgb_original)
img_yuv = cv2.cvtColor(img_rgb_original, cv2.COLOR_BGR2YUV)
img_yuv[:, :, 0] = merged  # replace luminance with pencil drawing
img_rgb_result = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
cv2.imwrite(os.path.join(output_path, "result_RGB.jpg"), img_rgb_result)
# (Visualization windows are disabled in integrated code)
# cv2.imshow("result", merged); cv2.imshow("RGB", img_rgb_result); cv2.waitKey(0)

print("Final pencil drawing images saved to output folder.")

# --------------- Step 5: Initialize MoveIt2 and plan robot motions to draw strokes ---------------
print("Initializing MoveIt2 for robotic drawing...")
rclpy.init(args=None)
moveit2 = MoveIt2(node_name="draw_node",
                  planning_group=planning_group_name,
                  base_link_name=base_frame,
                  end_effector_name=None)  # end_effector_name can be None if default end effector is used

# Optionally, set planner parameters (e.g., velocity scaling, tolerances) if needed
# moveit2.set_max_velocity(0.1)
# moveit2.set_max_acceleration(0.1)
# moveit2.set_pose_goal_tolerance(0.005)

# Pre-compute some values for coordinate transformation
cx = w0 / 2.0  # image center x
cy = h0 / 2.0  # image center y
scale_x = drawing_width / float(w0)
scale_y = drawing_height / float(h0)
z_down = plane_height
z_up = plane_height + lift_distance

print(f"Image size: {w0}x{h0} pixels. Drawing area: {drawing_width}m x {drawing_height}m. Scale: {scale_x:.4f} m/pixel (X), {scale_y:.4f} m/pixel (Y).")

# Iterate through each stroke in the sequence and command the robot
stroke_count = 0
for stroke in stroke_sequence:
    stroke_count += 1
    angle = stroke['angle']
    # Use precomputed rotation parameters for this angle
    nW, nH, cosA, sinA = angle_params.get(angle, (None, None, None, None))
    if nW is None:
        # Compute if missing (should not happen if angle_params was filled above)
        theta = math.radians(angle)
        nW = int(abs(w0 * math.cos(theta)) + abs(h0 * math.sin(theta)))
        nH = int(abs(w0 * math.sin(theta)) + abs(h0 * math.cos(theta)))
        cosA = math.cos(math.radians(angle))
        sinA = math.sin(math.radians(angle))
        angle_params[angle] = (nW, nH, cosA, sinA)
    # Determine the center of the stroke in rotated coords
    y_r_center = stroke['row'] - period  # approximate center row of the stroke in rotated image coordinates
    x_r_begin = stroke['begin']
    x_r_end   = stroke['end']
    # Compute original image coordinates (px, py) for the stroke endpoints
    px1 = cosA * x_r_begin + sinA * y_r_center - cosA * (nW / 2.0) - sinA * (nH / 2.0) + cx
    py1 = -sinA * x_r_begin + cosA * y_r_center + sinA * (nW / 2.0) - cosA * (nH / 2.0) + cy
    px2 = cosA * x_r_end + sinA * y_r_center - cosA * (nW / 2.0) - sinA * (nH / 2.0) + cx
    py2 = -sinA * x_r_end + cosA * y_r_center + sinA * (nW / 2.0) - cosA * (nH / 2.0) + cy
    # Convert image coordinates to robot coordinates (in base_frame)
    # Here we map image center to (0,0) in robot plane, x right, y up:
    rx1 = (px1 - cx) * scale_x + 0.0 + (base_x_offset := 0.0)  # you can set base_x_offset if needed
    ry1 = (cy - py1) * scale_y + 0.0 + (base_y_offset := 0.0)  # base_y_offset can shift the drawing in Y
    rx2 = (px2 - cx) * scale_x + base_x_offset
    ry2 = (cy - py2) * scale_y + base_y_offset
    # Define start and end poses for the stroke
    pose_start_up = PoseStamped()
    pose_start_up.header.frame_id = base_frame
    pose_start_up.pose.position.x = rx1
    pose_start_up.pose.position.y = ry1
    pose_start_up.pose.position.z = z_up
    pose_start_up.pose.orientation.x = target_orientation[0]
    pose_start_up.pose.orientation.y = target_orientation[1]
    pose_start_up.pose.orientation.z = target_orientation[2]
    pose_start_up.pose.orientation.w = target_orientation[3]
    pose_start_down = PoseStamped()
    pose_start_down.header.frame_id = base_frame
    pose_start_down.pose.position.x = rx1
    pose_start_down.pose.position.y = ry1
    pose_start_down.pose.position.z = z_down
    pose_start_down.pose.orientation = pose_start_up.pose.orientation  # same orientation
    pose_end_down = PoseStamped()
    pose_end_down.header.frame_id = base_frame
    pose_end_down.pose.position.x = rx2
    pose_end_down.pose.position.y = ry2
    pose_end_down.pose.position.z = z_down
    pose_end_down.pose.orientation = pose_start_up.pose.orientation  # same orientation
    pose_end_up = PoseStamped()
    pose_end_up.header.frame_id = base_frame
    pose_end_up.pose.position.x = rx2
    pose_end_up.pose.position.y = ry2
    pose_end_up.pose.position.z = z_up
    pose_end_up.pose.orientation = pose_start_up.pose.orientation  # same orientation

    # Move robot to the start of the stroke (above the paper)
    moveit2.move_to_pose(pose_start_up)
    # Lower the pen to the drawing surface
    moveit2.move_to_pose(pose_start_down)
    # Plan and execute a Cartesian path from start_down to end_down (drawing the line)
    # If using MoveIt2 Python API, plan_cartesian_path expects a list of Pose or PoseStamped waypoints.
    trajectory = moveit2.plan_cartesian_path([pose_end_down], avoid_collisions=True)  # plan a straight line path
    moveit2.execute(trajectory)
    # Lift the pen up at the end of the stroke
    moveit2.move_to_pose(pose_end_up)
    print(f"Stroke {stroke_count}/{len(stroke_sequence)} drawn (Angle {angle:.1f}°, length {stroke['end']-stroke['begin']} pixels).")

print("All strokes executed by robot.")
# Shutdown ROS2 node
rclpy.shutdown()
