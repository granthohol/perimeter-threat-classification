# AI-Driven Sensor Fusion for Autonomous Perimeter Defense

**A portfolio project by Grant Hohol**

---

## 📋 Overview

Fuses camera + LiDAR data to classify and localize perimeter threats in real time.  
Implements:
- C++ fusion pipeline (OpenCV + PCL)
- PyTorch-based object classification
- Threat scoring (confidence, distance, velocity)
- Flask dashboard for visualization
- Mock Palantir AIP ingestion layer

---

## 📂 Project Structure
```txt
perimeter-threat-classification/
├── data/ # Raw and output data (ignored by Git)
├── fusion/ # C++ fusion node
├── ml/ # Training and inference scripts
├── visualization/ # Flask dashboard
├── palantir_integration/ # Ontology + ingestion simulation
├── README.md
└── .gitignore
```

---

## 🚀 Getting Started

### Prerequisites

- Ubuntu 22.04 LTS  
- NVIDIA GPU + CUDA drivers  
- GCC, CMake  
- ROS 2 Humble  
- Python 3.10, uv, torch, torchvision, opencv-python, flask  
- OpenCV, PCL libraries  

### Setup

```bash
# Clone and enter project
cd ~/projects
git clone <your-repo-url> sensor_fusion_project
cd sensor_fusion_project

# Install system deps
sudo apt update && sudo apt install -y build-essential cmake libopencv-dev libpcl-dev ros-humble-desktop python3-pip python3-venv

# Create Python venv
python3 -m venv .venv
source .venv/bin/activate

# Install Python deps via uv
uv pip install torch torchvision opencv-python flask
```

## 🛠️ Usage

1. Build fusion pipeline

    ```bash
    mkdir build && cd build
    cmake ..
    make

    ```

2. Run fusion node
    ```bash
    ./sensor_fusion
    ```

3. Train ML model 
    ```bash
    python3 ml/train_model.py
    ```

4. Run inference
```bash
python3 ml/inference.py
```

5. Launch dashboard
```bash
python3 visualization/app.py
```

6. Simulate AIP ingestion
```bash
python3 palantir_integration/ingest_to_mock_aip.py
```

## 📈 Future Work

- Kalman filters for multi-target tracking
- Add radar modality