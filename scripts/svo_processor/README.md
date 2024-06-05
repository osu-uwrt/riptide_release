# SVO to AVI Export and Video Processing Script

## Overview

This script processes ZED SVO files, converting them to AVI format and rectifying the videos using calibration data. It handles the following tasks:

- Converts SVO files to side-by-side stereo AVI videos.
- Extracts and rectifies the left frames from the stereo videos based on provided calibration data.
- Saves the processed videos to specified directories.

## Usage

### Configuration Parameters

| Parameter Name   | Type    | Default                   | Description                                           |
|------------------|---------|---------------------------|-------------------------------------------------------|
| `calib_file`     | `str`   | `../config/ZED_cal.conf` | Path to the ZED calibration file                      |
| `svo_dir`        | `str`   | `../svos`                 | Directory containing the SVO files                    |
| `stereo_dir`     | `str`   | `../stereo_raw`           | Directory to save the raw stereo videos               |
| `left_dir`       | `str`   | `../left_raw`                 | Directory to save the extracted unrectified left frames |
| `rectified_dir`  | `str`   | `../left_rectified`            | Directory to save the rectified videos                |
| `save_left_raw`  | `bool`  | `False`                   | Boolean flag to save the unrectified left frames separately |

### Running the Script

1. Ensure your `config.yml` is set up correctly in the repository.
2. Run the script:

   ```bash
   python process_svo.py
   ```

This will:

- Convert SVO files to AVI format.
- Extract and rectify left frames.
- Save the processed videos to the specified directories.
