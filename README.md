# SIH Search-and-Rescue: 3D Scene Reconstruction from Monocular RGB

[![Python 3.10+](https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12%20%7C%203.13-blue.svg)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20Raspberry%20Pi-brightgreen.svg)]()
[![Inference](https://img.shields.io/badge/runtime-ONNX%20Runtime%20(CPU)-orange.svg)]()
[![Docker](https://img.shields.io/badge/docker-compatible-blue.svg)]()

A standalone, modular Python system developed for Search-and-Rescue (SAR) disaster site situational awareness. It ingests single RGB aerial or ground perspective imagery and produces metric-scaled depth estimates and full-color 3D point clouds (`.ply`) for interactive geometric inspection.

---

## ⚠️ Important Technical Distinction: Monocular Depth vs. LiDAR

> **Crucial Engineering Clarification:**  
> This application does **NOT** claim that RGB imagery is converted into real hardware LiDAR data.
>
> * **Hardware LiDAR** utilizes physical lasers firing pulsed beams at nanosecond intervals to calculate time-of-flight (ToF) direct range measurements.
> * **This System** uses deep-learning **Monocular Depth Estimation (MDE)** via lightweight convolutional networks (`MiDaS v2.1 Small`) to infer optical disparity from 2D scene geometry, lighting, and semantic perspective cues.
> * The estimated depth is subsequently back-projected into 3D Euclidean space $(X, Y, Z)$ using a **pinhole camera intrinsic projection model**, generating a metric point cloud while preserving true RGB vertex texturing.

---

## Key Features

* **CPU-First & Edge-Ready:** Runs out of the box on standard x86 CPUs and edge devices such as the **Raspberry Pi 4 (4 GB RAM)** using ONNX Runtime without requiring dedicated GPUs.
* **Modular, Decoupled Architecture:**
  * Depth estimation model is abstracted behind `BaseDepthModel`, allowing zero-friction swapping for future models (e.g., Depth-Anything-V2, FastDepth).
  * 3D Point-cloud reconstruction is isolated from inference logic and handles camera intrinsics (FOV or calibration matrix).
  * Visualization is separated from processing with automatic **headless detection** for servers and Docker environments.
* **Lossless Outputs:**
  1. `original.png`: Reference input image.
  2. `depth.png`: Color-mapped false-color depth visualization (Inferno/Turbo colormap).
  3. `depth_raw.png`: Lossless 16-bit millimeter-scale depth map.
  4. `pointcloud.ply`: Dense 3D point cloud with RGB vertex colors.
* **Point Cloud Post-Processing:** Built-in voxel downsampling and statistical outlier removal to eliminate depth edge noise and flying pixels.
* **Extensible Input:** Pinhole parameters are decoupled from physical sensors, allowing direct migration to Raspberry Pi Camera Module v2/v3 or video frame streams.

---

## Project Structure

```text
sih26177/
├── app/
│   ├── depth/
│   │   ├── __init__.py
│   │   ├── model.py            # BaseDepthModel, MiDaSSmallONNX, DummyDepthModel
│   │   └── inference.py        # DepthEstimator pipeline & disparity-to-depth conversion
│   ├── pointcloud/
│   │   ├── __init__.py
│   │   └── reconstruction.py   # CameraIntrinsics, Pinhole back-projection, PLY writer
│   ├── visualization/
│   │   ├── __init__.py
│   │   └── viewer.py           # Interactive Open3D visualizer & headless handler
│   └── main.py                 # CLI orchestration entrypoint
├── config/
│   └── config.yaml             # Camera, model, voxel, and output configuration
├── data/
│   ├── input/
│   │   └── image.jpg           # Disaster/rescue scene sample image
│   └── output/
│       ├── original.png
│       ├── depth.png
│       ├── depth_raw.png
│       └── pointcloud.ply
├── models/
│   └── model-small.onnx        # Auto-downloaded MiDaS v2.1 Small weights (~45 MB)
├── tests/
│   ├── test_reconstruction.py  # Pinhole geometry math, depth filtering, PLY export tests
│   ├── test_depth.py           # Depth model interface and metric scaling tests
│   └── test_pipeline.py        # End-to-end integration test
├── Dockerfile                  # Containerized deployment specification
├── requirements.txt            # Python dependencies
└── README.md
```

---

## Installation & Setup

### 1. Prerequisites
* Python 3.10, 3.11, 3.12, or 3.13.
* Git.

### 2. Create Virtual Environment

**Windows (PowerShell):**
```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

**Linux / macOS / Raspberry Pi OS:**
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

*(Note: On Debian/Ubuntu/Raspberry Pi OS, ensure system OpenGL libraries are present: `sudo apt-get install -y libgl1 libglib2.0-0 libgomp1`)*

---

## Quickstart & Usage

### 1. Run Pipeline with Default Sample Image

```bash
python -m app.main --input data/input/image.jpg
```
* On the first run, the lightweight pretrained MiDaS v2.1 Small model (~45 MB) will automatically download into `models/model-small.onnx`.
* The pipeline will estimate the depth, reconstruct the 3D colored point cloud, save all artifacts to `data/output/`, and open an interactive Open3D 3D viewer.

### 2. Headless Mode (For CI/CD, SSH, or Docker)

```bash
python -m app.main --input data/input/image.jpg --headless
```

### 3. Custom Output Directory and Configuration

```bash
python -m app.main --input path/to/drone_capture.jpg --output my_output_dir/ --config config/config.yaml
```

### 4. Overriding Camera FOV & Voxel Resolution

For a Raspberry Pi Camera Module 2 (horizontal FOV $\approx 62.2^\circ$):
```bash
python -m app.main --input data/input/image.jpg --fov 62.2 --voxel-size 0.03
```

---

## Controls for Interactive 3D Viewer

When the Open3D viewer launches:
* **Rotate:** Left Click + Mouse Drag
* **Pan:** Right Click + Mouse Drag (or Shift + Left Click)
* **Zoom:** Mouse Scroll Wheel
* **Exit:** Press `Q` or `ESC`

---

## Inspecting the Output Point Cloud

The output `data/output/pointcloud.ply` preserves RGB vertex colors and can also be opened in external 3D software:
* [CloudCompare](https://www.danielgm.net/cc/) (Free, open-source 3D point cloud editor)
* [MeshLab](https://www.meshlab.net/) (Free, open-source 3D mesh & point processor)
* Blender (`File -> Import -> Stanford (.ply)`)

---

## Docker Deployment

To build and run inside a self-contained Docker container:

```bash
# Build the Docker image
docker build -t sih-3d-reconstruct .

# Run container on a sample image (mounting output folder)
docker run --rm -v ${PWD}/data/output:/workspace/data/output sih-3d-reconstruct
```

---

## Running the Automated Test Suite

Run unit and integration tests using `pytest`:

```bash
pytest -v
```

All tests run deterministically and do not depend on active internet connectivity (using `DummyDepthModel` mocks for depth pipeline verification).

---

## Extending for Future Milestones

1. **Raspberry Pi Camera Stream:**
   * The `DepthEstimator.estimate_frame(frame)` interface accepts direct numpy buffers from `cv2.VideoCapture(0)` or `picamera2`.
2. **Video & Trajectory Processing:**
   * Downstream modules can accumulate successive point clouds into a global coordinate frame via ICP (Iterative Closest Point) registration.
3. **Drone & MAVLink Integration:**
   * Telemetry (roll, pitch, yaw, altitude) can be passed directly as extrinsic transformation matrices to orient the point cloud in real-world geospatial coordinates.
