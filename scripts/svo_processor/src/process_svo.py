import sys
import pyzed.sl as sl
import numpy as np
import cv2
import os
import yaml
from pathlib import Path

def progress_bar(percent_done, bar_length=50):
    done_length = int(bar_length * percent_done / 100)
    bar = '=' * done_length + '-' * (bar_length - done_length)
    sys.stdout.write('[%s] %i%%\r' % (bar, percent_done))
    sys.stdout.flush()

def svo_to_avi_export(svo_dir, stereo_dir):

    os.makedirs(stereo_dir, exist_ok=True)

    svo_files = list(Path(svo_dir).glob('*.svo')) + list(Path(svo_dir).glob('*.svo2'))

    if not svo_files:
        sys.stdout.write(f"No .svo files found in {svo_dir}\n")
        return
    
    for svo_file in svo_files:
        svo_input_path = str(svo_file)
        # Changed to .mkv for better lossless codec support
        mkv_output_path = os.path.join(stereo_dir, svo_file.stem + '.mkv')

        if os.path.isfile(mkv_output_path):
            sys.stdout.write(f"Output file {mkv_output_path} already exists. Skipping...\n")
            continue

        init_params = sl.InitParameters()
        init_params.set_from_svo_file(svo_input_path)
        init_params.svo_real_time_mode = False
        init_params.coordinate_units = sl.UNIT.MILLIMETER

        zed = sl.Camera()
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            sys.stdout.write(f"Error opening {svo_input_path}. Skipping...\n")
            zed.close()
            continue

        image_size = zed.get_camera_information().camera_configuration.resolution
        width = image_size.width
        height = image_size.height
        width_sbs = width * 2

        svo_image_sbs_rgba = np.zeros((height, width_sbs, 4), dtype=np.uint8)
        left_image = sl.Mat()
        right_image = sl.Mat()

        # Using FFV1 lossless codec
        video_writer = cv2.VideoWriter(mkv_output_path,
                                       cv2.VideoWriter_fourcc('F', 'F', 'V', '1'),
                                       max(zed.get_camera_information().camera_configuration.fps, 25),
                                       (width_sbs, height))
        if not video_writer.isOpened():
            sys.stdout.write(f"OpenCV video writer cannot be opened for {mkv_output_path}\n")
            zed.close()
            continue

        rt_param = sl.RuntimeParameters()
        sys.stdout.write(f"Converting {svo_input_path} to {mkv_output_path}... Use Ctrl-C to interrupt conversion.\n")

        nb_frames = zed.get_svo_number_of_frames()

        while True:
            err = zed.grab(rt_param)
            if err == sl.ERROR_CODE.SUCCESS:
                svo_position = zed.get_svo_position()
                zed.retrieve_image(left_image, sl.VIEW.LEFT)
                zed.retrieve_image(right_image, sl.VIEW.RIGHT)
                svo_image_sbs_rgba[0:height, 0:width, :] = left_image.get_data()
                svo_image_sbs_rgba[0:, width:, :] = right_image.get_data()
                ocv_image_sbs_rgb = cv2.cvtColor(svo_image_sbs_rgba, cv2.COLOR_RGBA2RGB)
                video_writer.write(ocv_image_sbs_rgb)
                progress_bar((svo_position + 1) / nb_frames * 100, 30)

            if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                progress_bar(100, 30)
                break

        video_writer.release()
        zed.close()
    sys.stdout.write("All SVO files have been processed. \n")

def load_calibration_data_conf(calib_file, resolution):
    """Load calibration data from ZED .conf file format"""
    with open(calib_file, 'r') as file:
        calib = file.read().splitlines()

    calib_data = {}
    section = None

    for line in calib:
        if line.startswith('[') and line.endswith(']'):
            section = line[1:-1]
            calib_data[section] = {}
        elif '=' in line and section:
            key, value = line.split('=')
            calib_data[section][key.strip()] = float(value.strip())

    cam_section = f'LEFT_CAM_{resolution}'

    if cam_section not in calib_data:
        raise ValueError(f"Calibration data for {resolution} not found in {calib_file}")

    params = calib_data[cam_section]
    camera_matrix = np.array([[params['fx'], 0, params['cx']],
                              [0, params['fy'], params['cy']],
                              [0, 0, 1]], dtype=np.float32)

    dist_coeffs = np.array([params['k1'], params['k2'], params['p1'], params['p2'], params.get('k3', 0)], dtype=np.float32)

    return camera_matrix, dist_coeffs

def load_calibration_data_yaml(calib_file):
    """Load calibration data from OpenCV YAML file format using cv2.FileStorage"""
    fs = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
    
    if not fs.isOpened():
        raise ValueError(f"Could not open calibration file: {calib_file}")
    
    # Read camera matrix and distortion coefficients
    camera_matrix = fs.getNode("K_LEFT").mat()
    dist_coeffs = fs.getNode("D_LEFT").mat().flatten()  # Flatten to 1D array
    
    # Try to read the calibration image size - handle different storage formats
    calib_size = None
    try:
        size_node = fs.getNode("Size")
        if not size_node.empty():
            if size_node.isSeq():
                # Size stored as sequence [width, height]
                width = int(size_node.at(0).real())
                height = int(size_node.at(1).real())
                calib_size = (width, height)
            elif size_node.isMat():
                # Size stored as matrix
                size_data = size_node.mat()
                if size_data is not None and size_data.size >= 2:
                    calib_size = (int(size_data[0]), int(size_data[1]))
    except Exception as e:
        sys.stdout.write(f"Warning: Could not read Size from YAML file: {e}\n")
        calib_size = None
    
    fs.release()
    
    if camera_matrix is None or dist_coeffs is None:
        raise ValueError(f"Could not read K_LEFT or D_LEFT from {calib_file}")
    
    return camera_matrix.astype(np.float32), dist_coeffs.astype(np.float32), calib_size

def scale_camera_matrix(camera_matrix, original_size, target_size):
    """Scale camera matrix from original calibration size to target video size"""
    if original_size is None:
        sys.stdout.write("Warning: No calibration size found in YAML file. Assuming calibration matches video resolution.\n")
        return camera_matrix
    
    orig_w, orig_h = original_size
    target_w, target_h = target_size
    
    # Calculate scaling factors
    scale_x = target_w / orig_w
    scale_y = target_h / orig_h
    
    if abs(scale_x - 1.0) < 0.01 and abs(scale_y - 1.0) < 0.01:
        # No significant scaling needed
        return camera_matrix
    
    sys.stdout.write(f"Scaling camera matrix from {original_size} to {target_size} (scale: {scale_x:.3f}, {scale_y:.3f})\n")
    
    # Scale the camera matrix
    scaled_matrix = camera_matrix.copy()
    scaled_matrix[0, 0] *= scale_x  # fx
    scaled_matrix[1, 1] *= scale_y  # fy
    scaled_matrix[0, 2] *= scale_x  # cx
    scaled_matrix[1, 2] *= scale_y  # cy
    
    return scaled_matrix

def load_calibration_data(calib_file, resolution=None):
    """Load calibration data from either .conf or .yaml/.yml file"""
    file_extension = Path(calib_file).suffix.lower()
    
    if file_extension == '.conf':
        if resolution is None:
            raise ValueError("Resolution must be specified for .conf files")
        camera_matrix, dist_coeffs = load_calibration_data_conf(calib_file, resolution)
        return camera_matrix, dist_coeffs, None  # No size info from .conf files
    elif file_extension in ['.yaml', '.yml']:
        return load_calibration_data_yaml(calib_file)  # Returns camera_matrix, dist_coeffs, calib_size
    else:
        raise ValueError(f"Unsupported calibration file format: {file_extension}")

def process_video(input_path, left_output_path, rectified_output_path, camera_matrix, dist_coeffs, save_left_raw):
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        sys.stdout.write(f"Error opening video file: {input_path}\n")
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    # Using FFV1 lossless codec for both outputs
    lossless_codec = cv2.VideoWriter_fourcc('F', 'F', 'V', '1')

    # Handle case where frame count is unreliable
    use_frame_count = total_frames > 0
    if not use_frame_count:
        sys.stdout.write(f"Warning: Unable to get frame count for {input_path}. Progress will be estimated.\n")

    if save_left_raw:
        out_left = cv2.VideoWriter(left_output_path, lossless_codec, fps, (width // 2, height))

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (width // 2, height), 0)
    out_rectified = cv2.VideoWriter(rectified_output_path, lossless_codec, fps, (roi[2], roi[3]))

    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        left_half = frame[:, :width // 2]
        
        if save_left_raw:
            out_left.write(left_half)

        undistorted_frame = cv2.undistort(left_half, camera_matrix, dist_coeffs, None, new_camera_matrix)
        x, y, w, h = roi
        undistorted_frame = undistorted_frame[y:y+h, x:x+w]
        out_rectified.write(undistorted_frame)

        frame_count += 1
        
        # Show progress based on whether we have reliable frame count
        if use_frame_count:
            progress_bar((frame_count / total_frames) * 100)
        else:
            # Show progress without percentage
            if frame_count % 30 == 0:  # Update every 30 frames
                sys.stdout.write(f"\rProcessed {frame_count} frames...")
                sys.stdout.flush()

    cap.release()
    if save_left_raw:
        out_left.release()
        sys.stdout.write(f"\nProcessed video saved to: {left_output_path}")

    out_rectified.release()
    sys.stdout.write(f"\nRectified video saved to: {rectified_output_path}\n")
    sys.stdout.write(f"Total frames processed: {frame_count}\n")

def load_camera_params_and_paths(config_path):
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
        calib_file = config.get('calib_file', '../config/ZED_cal.conf')
        svo_dir = config.get('svo_dir', '../svos')
        stereo_dir = config.get('stereo_dir', '../stereo_raw')
        left_dir = config.get('left_dir', '../left_raw')
        rectified_dir = config.get('rectified_dir', '../left_rectified')
        save_left_raw = config.get('save_left_raw', False)
    
    return calib_file, svo_dir, stereo_dir, left_dir, rectified_dir, save_left_raw

def determine_resolution(width):
    if width == 4416:
        return '2K'
    elif width == 3840:
        return 'FHD'
    elif width == 2560:
        return 'HD'
    elif width == 1344:
        return 'VGA'
    else:
        raise ValueError(f"Unsupported resolution width: {width}")

def process_directory(stereo_dir, left_dir, rectified_dir, calib_file, save_left_raw):
    if not os.path.exists(left_dir):
        os.makedirs(left_dir)
    if not os.path.exists(rectified_dir):
        os.makedirs(rectified_dir)

    # Check if calibration file is .conf or .yaml format
    file_extension = Path(calib_file).suffix.lower()
    is_conf_format = file_extension == '.conf'

    # Updated to handle both .avi and .mkv files
    for filename in os.listdir(stereo_dir):
        if filename.endswith((".avi", ".mkv")):
            input_video_path = os.path.join(stereo_dir, filename)
            left_output_path = os.path.join(left_dir, filename) if save_left_raw else None
            rectified_output_path = os.path.join(rectified_dir, filename)
            
            if os.path.exists(rectified_output_path):
                sys.stdout.write(f"File {filename} already exists in rectified folder. Skipping...\n")
                continue

            cap = cv2.VideoCapture(input_video_path)
            if not cap.isOpened():
                sys.stdout.write(f"Error opening video file: {input_video_path}\n")
                continue
            
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            cap.release()
            
            # Load calibration data based on file format
            if is_conf_format:
                resolution = determine_resolution(width)
                camera_matrix, dist_coeffs, calib_size = load_calibration_data(calib_file, resolution)
            else:
                # For YAML files, we get the calibration size from the file
                camera_matrix, dist_coeffs, calib_size = load_calibration_data(calib_file)
                
                # Scale camera matrix if needed (for left half of stereo image)
                left_width = width // 2
                video_size = (left_width, height)
                camera_matrix = scale_camera_matrix(camera_matrix, calib_size, video_size)
            
            sys.stdout.write(f"\nProcessing video: {filename}\n")
            process_video(input_video_path, left_output_path, rectified_output_path, camera_matrix, dist_coeffs, save_left_raw)

if __name__ == "__main__":
    config_path = '../config/config.yml'
    calib_file, svo_dir, stereo_dir, left_dir, rectified_dir, save_left_raw = load_camera_params_and_paths(config_path)
    svo_to_avi_export(svo_dir, stereo_dir)
    process_directory(stereo_dir, left_dir, rectified_dir, calib_file, save_left_raw)
