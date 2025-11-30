# Hardware Setup Guide - Jetson Orin Nano

**Complete guide to set up and deploy the Elderly Companion Robot on NVIDIA Jetson Orin Nano hardware.**

**✅ Your Jetson is already flashed and accessible? Great! Start with Step 1 below.**

---

## 📋 Prerequisites

Before starting, make sure you have:
- ✅ NVIDIA Jetson Orin Nano 8GB Developer Kit (already flashed)
- ✅ Jetson is accessible via SSH or direct connection
- ✅ All robot components (chassis, motors, camera, etc.)
- ✅ Official Jetson power adapter

---

## Step 1: Initial Jetson Configuration (15 minutes)

### 1.1 Update System

```bash
# Update package lists
sudo apt update
sudo apt upgrade -y

# Install essential tools
sudo apt install -y \
    git \
    curl \
    wget \
    python3-pip \
    python3-venv \
    build-essential \
    cmake \
    pkg-config
```

### 1.2 Install Python 3.11

```bash
# Install Python 3.11
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt update
sudo apt install -y python3.11 python3.11-venv python3.11-dev

# Verify
python3.11 --version
```

### 1.3 Install Docker

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group
sudo usermod -aG docker $USER

# Log out and back in, or:
newgrp docker

# Verify
docker --version
docker compose version
```

### 1.4 Install ROS 2 Humble (Optional - for robot control)

```bash
# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2
sudo apt update
sudo apt install -y ros-humble-ros-base

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify
ros2 --help
```

---

## Step 2: Transfer Project to Jetson (10 minutes)

### Option 1: Git Clone (Recommended)

```bash
# On Jetson
cd ~
git clone <your-repo-url> ai-project
# Or if using SSH from your Mac:
# git clone git@github.com:yourusername/elderly-companion.git ai-project

cd ~/ai-project/elderly_companion
```

### Option 2: SCP Transfer from Mac

```bash
# On your Mac
cd ~/Documents/ops/ai-project
scp -r elderly_companion jetson@<jetson-ip>:~/

# On Jetson
cd ~/elderly_companion
```

### Option 3: USB Drive

1. Copy `elderly_companion` folder to USB drive
2. Plug into Jetson
3. Copy to home directory

---

## Step 3: Set Up Python Environment on Jetson (10 minutes)

### 3.1 Create Virtual Environment

```bash
cd ~/elderly_companion/ai_services

# Create venv with Python 3.11
python3.11 -m venv venv

# Activate
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip setuptools wheel
```

### 3.2 Install Dependencies

**Important:** Install PyTorch for Jetson (CUDA-enabled)

```bash
# Install PyTorch for Jetson (CUDA-enabled)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install other requirements
pip install -r requirements_speech.txt
pip install -r requirements_llm.txt
pip install -r requirements_vision.txt

# Install httpx for Ollama
pip install httpx
```

### 3.3 Install Ollama

```bash
# Download and install Ollama
curl -fsSL https://ollama.ai/install.sh | sh

# Start Ollama service
sudo systemctl enable ollama
sudo systemctl start ollama

# Download model (use smaller 1B model for Jetson)
ollama pull llama3.2:1b
```

---

## Step 4: Hardware Assembly (30-60 minutes)

### 4.1 Mount Jetson on Chassis

1. **Secure Jetson:**
   - Use mounting holes on Jetson
   - Attach to chassis with standoffs
   - Ensure good airflow

2. **Connect Motor Controller:**
   - Attach motor driver HAT to Jetson GPIO
   - Connect motors to driver
   - Test motor connections

### 4.2 Connect Sensors

1. **Camera:**
   ```bash
   # Test camera
   ls /dev/video*
   # Should see /dev/video0 or similar
   
   # Test with v4l2
   sudo apt install -y v4l-utils
   v4l2-ctl --list-devices
   ```

2. **Microphone/Speaker:**
   ```bash
   # Test audio
   arecord -l  # List recording devices
   aplay -l    # List playback devices
   ```

3. **LIDAR (if using):**
   - Connect via USB
   - Check device: `ls /dev/ttyUSB*`

### 4.3 Power Setup

1. **Connect Power:**
   - Use official Jetson power adapter
   - Connect to power bank or wall outlet
   - Ensure stable power supply

2. **Test Power:**
   ```bash
   # Check power status
   sudo tegrastats
   # Should show power consumption
   ```

---

## Step 5: Configure Services for Jetson (15 minutes)

### 5.1 Update Service Scripts

The services should work as-is, but you may want to optimize:

**For Vision Service (use GPU):**
```python
# In vision_service_enhanced.py, change:
device='cuda'  # Instead of 'cpu'
```

**For LLM Service:**
- Use smaller model: `llama3.2:1b` (already configured)
- Monitor memory usage

### 5.2 Create Systemd Services (Optional - Auto-start)

```bash
# Create service file for AI services
sudo nano /etc/systemd/system/companion-ai.service
```

Add:
```ini
[Unit]
Description=Elderly Companion AI Services
After=network.target docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/home/jetson/elderly_companion/ai_services
ExecStart=/usr/bin/docker compose up -d
ExecStop=/usr/bin/docker compose down
User=jetson

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl enable companion-ai
sudo systemctl start companion-ai
```

---

## Step 6: Test Everything (30 minutes)

### 6.1 Test Services Locally

```bash
# Activate virtual environment
cd ~/elderly_companion/ai_services
source venv/bin/activate

# Test speech service
python scripts/speech_service.py

# Test vision service (in another terminal)
python scripts/vision_service_enhanced.py

# Test LLM service (in another terminal)
python scripts/llm_service_ollama.py
```

### 6.2 Test Docker Services

```bash
cd ~/elderly_companion/ai_services

# Build and start services
docker compose build
docker compose up -d

# Check status
docker compose ps

# View logs
docker compose logs -f [service_name]

# Test endpoints
curl http://localhost:8001/health  # Speech
curl http://localhost:8002/health  # LLM
curl http://localhost:8003/health  # Vision
```

### 6.3 Test ROS 2 Bridge (if ROS 2 installed)

```bash
cd ~/elderly_companion/ros2_workspace

# Build package
colcon build --packages-select companion_bridge

# Source workspace
source install/setup.bash

# Run bridge
ros2 run companion_bridge ai_bridge

# In another terminal, test topics
ros2 topic list
ros2 topic echo /ai/vision/detections
```

---

## Step 7: Performance Optimization (Optional)

### 7.1 Set Power Mode

```bash
# Set to maximum performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Check current mode
sudo nvpmodel -q
```

### 7.2 Optimize Docker Resources

Edit `docker-compose.yml` to adjust resource limits:

```yaml
services:
  vision:
    deploy:
      resources:
        limits:
          cpus: '4'
          memory: 4G
```

### 7.3 Monitor Performance

```bash
# Monitor system resources
sudo tegrastats

# Monitor Docker resources
docker stats

# Monitor GPU usage
sudo jetson_stats
```

---

## Step 8: Deployment Checklist

- [ ] Jetson is flashed and accessible
- [ ] System updated and essential tools installed
- [ ] Python 3.11 installed
- [ ] Docker installed and user added to docker group
- [ ] ROS 2 Humble installed (if needed)
- [ ] Project transferred to Jetson
- [ ] Virtual environment created and dependencies installed
- [ ] Ollama installed and model downloaded
- [ ] Hardware assembled and sensors connected
- [ ] Services tested locally
- [ ] Docker services tested
- [ ] ROS 2 bridge tested (if applicable)
- [ ] Performance optimized (optional)
- [ ] Systemd services configured (optional)

---

## 🆘 Troubleshooting

### Docker Permission Denied

```bash
# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

### Camera Not Detected

```bash
# Check camera devices
ls /dev/video*

# Install v4l2 tools
sudo apt install -y v4l-utils
v4l2-ctl --list-devices
```

### Out of Memory

```bash
# Check memory usage
free -h

# Reduce Docker memory limits in docker-compose.yml
# Use smaller Ollama model (1B instead of 3B)
```

### Services Not Starting

```bash
# Check Docker logs
docker compose logs [service_name]

# Check system resources
sudo tegrastats
docker stats
```

---

## 📚 Next Steps

1. ✅ Complete hardware setup
2. ✅ Test all services
3. ✅ Deploy to production
4. ✅ Start developing!

**For Mac development setup, see:** [COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md)

---

**Need help?** Check the troubleshooting section above or refer to service-specific guides in `elderly_companion/ai_services/`.
