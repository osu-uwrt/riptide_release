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

    svo_files = [f for f in Path(svo_dir).glob('*.svo')]

    if not svo_files:
        sys.stdout.write(f"No .svo files found in {svo_dir}\n")
        return
    
    for svo_file in svo_files:
        svo_input_path = str(svo_file)
        avi_output_path = os.path.join(stereo_dir, svo_file.stem + '.avi')

        if os.path.isfile(avi_output_path):
            sys.stdout.write(f"Output file {avi_output_path} already exists. Skipping...\n")
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

        video_writer = cv2.VideoWriter(avi_output_path,
                                       cv2.VideoWriter_fourcc('M', '4', 'S', '2'),
                                       max(zed.get_camera_information().camera_configuration.fps, 25),
                                       (width_sbs, height))
        if not video_writer.isOpened():
            sys.stdout.write(f"OpenCV video writer cannot be opened for {avi_output_path}\n")
            zed.close()
            continue

        rt_param = sl.RuntimeParameters()
        sys.stdout.write(f"Converting {svo_input_path} to {avi_output_path}... Use Ctrl-C to interrupt conversion.\n")

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

def load_calibration_data(calib_file, resolution):
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

def process_video(input_path, left_output_path, rectified_output_path, camera_matrix, dist_coeffs, save_left_raw):
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        sys.stdout.write(f"Error opening video file: {input_path}\n")
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    codec = cv2.VideoWriter_fourcc(*'XVID')

    if save_left_raw:
        out_left = cv2.VideoWriter(left_output_path, codec, fps, (width // 2, height))

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (width // 2, height), 0)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out_rectified = cv2.VideoWriter(rectified_output_path, fourcc, fps, (roi[2], roi[3]))

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
        progress_bar((frame_count / total_frames) * 100)

    cap.release()
    if save_left_raw:
        out_left.release()
        sys.stdout.write(f"\nProcessed video saved to: {left_output_path}")

    out_rectified.release()
    sys.stdout.write(f"\nRectified video saved to: {rectified_output_path}\n")

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

    for filename in os.listdir(stereo_dir):
        if filename.endswith(".avi"):
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
            cap.release()
            
            resolution = determine_resolution(width)
            camera_matrix, dist_coeffs = load_calibration_data(calib_file, resolution)
            
            sys.stdout.write(f"\nProcessing video: {filename}\n")
            process_video(input_video_path, left_output_path, rectified_output_path, camera_matrix, dist_coeffs, save_left_raw)

if __name__ == "__main__":
    config_path = '../config/config.yml'
    calib_file, svo_dir, stereo_dir, left_dir, rectified_dir, save_left_raw = load_camera_params_and_paths(config_path)
    svo_to_avi_export(svo_dir, stereo_dir)
    process_directory(stereo_dir, left_dir, rectified_dir, calib_file, save_left_raw)
