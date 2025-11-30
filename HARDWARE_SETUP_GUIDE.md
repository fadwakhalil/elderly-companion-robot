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

# Enable universe repository (needed for build-essential)
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update

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

**If you get "unable to locate package build-essential" error:**

```bash
# Make sure universe repository is enabled
sudo add-apt-repository universe
sudo apt update

# Try installing again
sudo apt install -y build-essential

# If still not found, install components individually:
sudo apt install -y gcc g++ make libc6-dev
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

# Add ROS 2 GPG key (modern method - apt-key is deprecated)
# Download and convert the key to the proper format
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

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

# Upgrade pip and install essential packages
pip install --upgrade pip setuptools wheel
# Install pyyaml (required by launch-ros if ROS 2 is installed)
pip install pyyaml
```

### 3.2 Install Dependencies

**Important:** Install PyTorch for Jetson (CUDA-enabled) with compatible versions

```bash
# Option 1: Install PyTorch with specific versions that match requirements
# This ensures compatibility between torch, torchvision, and torchaudio
pip install torch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 --index-url https://download.pytorch.org/whl/cu118

# Option 2: If Option 1 fails, install requirements first (they specify versions)
# pip install -r requirements_speech.txt
# pip install -r requirements_llm.txt
# pip install -r requirements_vision.txt

# Install other requirements (PyTorch will be skipped if already installed)
pip install -r requirements_speech.txt
pip install -r requirements_llm.txt
pip install -r requirements_vision.txt

# Install httpx for Ollama (if not already in requirements)
pip install httpx
```

**If you get version conflicts, try this approach:**

```bash
# Uninstall any existing PyTorch packages
pip uninstall -y torch torchvision torchaudio

# Install PyTorch with compatible versions for Jetson
pip install torch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 --index-url https://download.pytorch.org/whl/cu118

# Then install requirements (use --no-deps to avoid reinstalling PyTorch)
pip install -r requirements_speech.txt --no-deps
pip install -r requirements_llm.txt
pip install -r requirements_vision.txt --no-deps

# Install any missing dependencies
pip install fastapi==0.104.1 uvicorn[standard]==0.24.0 pydantic==2.5.0
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

## Step 4: Test Services on Jetson (15-30 minutes)

**Before assembling hardware, verify all services work correctly on Jetson.**

### 4.1 Test Services Locally (Without Docker)

**Terminal 1 - Test Speech Service:**
```bash
cd ~/elderly_companion/ai_services
source venv/bin/activate

# Start speech service
python scripts/speech_service.py
```

**Expected Output:**
```
2024-XX-XX XX:XX:XX - __main__ - WARNING - ROS 2 not available - running without ROS 2 integration
INFO:     Started server process [XXXXX]
INFO:     Waiting for application startup.
2024-XX-XX XX:XX:XX - __main__ - INFO - Loading Whisper model: tiny.en
100%|████████████████████| 75/75 [00:XX<00:XX, X.XXit/s]
2024-XX-XX XX:XX:XX - __main__ - INFO - Model loaded successfully
2024-XX-XX XX:XX:XX - __main__ - INFO - Speech Service started successfully
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8001 (Press CTRL+C to quit)
```

**Note:** You should NOT see any deprecation warnings about `on_event`. The services now use the modern `lifespan` event handler.

**What to expect:**
- ✅ Service starts and loads Whisper model (may take 10-30 seconds first time)
- ✅ Server runs on `http://0.0.0.0:8001`
- ✅ Service stays running (don't close the terminal)
- ⚠️ Warning about ROS 2 is normal (if ROS 2 not installed)
- ✅ Model loading progress bar appears (first time only)

**Terminal 2 - Start Ollama Service (Required for LLM):**
```bash
# First, verify Ollama is installed
ollama --version

# Check if Ollama is already running
curl -s http://localhost:11434/api/tags > /dev/null && echo "Ollama is running" || echo "Ollama is not running"

# If not running, start Ollama service
# Option 1: Using systemd (recommended for Jetson)
sudo systemctl start ollama
sudo systemctl enable ollama  # Enable auto-start on boot

# Option 2: Run manually in foreground (for testing)
# ollama serve

# Verify Ollama is running
sleep 3
curl http://localhost:11434/api/tags
# Should return JSON with available models

# Check if model is downloaded
ollama list
# Should show llama3.2:1b or similar

# If model not downloaded, pull it:
ollama pull llama3.2:1b
```

**Terminal 3 - Test LLM Service:**
```bash
cd ~/elderly_companion/ai_services
source venv/bin/activate

# Start LLM service
python scripts/llm_service_ollama.py
```

**Expected Output:**
```
2024-XX-XX XX:XX:XX - __main__ - WARNING - ROS 2 not available - running without ROS 2 integration
INFO:     Started server process [XXXXX]
INFO:     Waiting for application startup.
2024-XX-XX XX:XX:XX - __main__ - INFO - Using Ollama with model: llama3.2:1b
2024-XX-XX XX:XX:XX - __main__ - INFO - Ollama host: http://localhost:11434
2024-XX-XX XX:XX:XX - __main__ - INFO - LLM Service (Ollama) started successfully
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8002 (Press CTRL+C to quit)
```

**Note:** If you see "Using rule-based responses (Ollama not available)" or "Ollama failed, falling back to rule-based", it means Ollama is not running or not accessible. The service will still work with rule-based responses, but for full functionality, start Ollama first.

**Terminal 4 - Test Vision Service:**
```bash
cd ~/elderly_companion/ai_services
source venv/bin/activate

# Start vision service
python scripts/vision_service_enhanced.py
```

**Test each service (in a new terminal):**
```bash
# Test Speech Service
curl http://localhost:8001/health
# Expected: {"status":"healthy","service":"speech","model_loaded":true,"ros2_available":false}

# Test LLM Service Health
curl http://localhost:8002/health
# Expected with Ollama: {"status":"healthy","service":"llm","personality_loaded":true,"ros2_available":false,"ollama_enabled":true,"ollama_status":"available","ollama_model":"llama3.2:1b"}
# Expected without Ollama: {"status":"healthy","service":"llm","personality_loaded":true,"ros2_available":false,"ollama_enabled":false,"ollama_status":"unavailable"}

# Test LLM chat
curl -X POST http://localhost:8002/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello, how are you?", "user_id": "test"}'

# Expected with Ollama: Natural conversation response
# Expected without Ollama (fallback): Rule-based response like "Hello there! I'm your companion. How are you feeling today?"

# Test Vision Service
curl http://localhost:8003/health
# Expected: {"status":"healthy","service":"vision","model_loaded":true,"ros2_available":false}
```

**Troubleshooting Ollama Connection:**

If you see `ollama request error: timed out` or `Ollama failed, falling back to rule-based`:

1. **Check if Ollama is running:**
   ```bash
   systemctl status ollama
   # OR
   curl http://localhost:11434/api/tags
   ```

2. **Start Ollama if not running:**
   ```bash
   sudo systemctl start ollama
   # Wait a few seconds for it to start
   sleep 5
   ```

3. **Verify Ollama is accessible:**
   ```bash
   curl http://localhost:11434/api/tags
   # Should return JSON, not an error
   ```

4. **Check if model is downloaded:**
   ```bash
   ollama list
   # Should show at least one model (e.g., llama3.2:1b)
   ```

5. **If model is missing, download it:**
   ```bash
   ollama pull llama3.2:1b
   ```

6. **Restart LLM service after Ollama is running:**
   ```bash
   # Stop the LLM service (Ctrl+C) and restart it
   python scripts/llm_service_ollama.py
   ```

**Note:** The service will work with rule-based fallback even if Ollama is not available. This is intentional for testing. However, for natural conversations, you need Ollama running.

### 4.2 Test Services with Docker

```bash
cd ~/elderly_companion/ai_services

# Stop any locally running services (Ctrl+C in their terminals)

# Build Docker images
docker compose build

# Start all services
docker compose up -d

# Check status
docker compose ps
# All services should show "Up" and "healthy"

# View logs
docker compose logs -f
# Press Ctrl+C to exit logs view

# Test all health endpoints
curl http://localhost:8001/health  # Speech
curl http://localhost:8002/health  # LLM
curl http://localhost:8003/health  # Vision
```

### 4.3 Test Service Functionality

**Test Speech Service:**
```bash
# Health check
curl http://localhost:8001/health

# Test transcription (if you have an audio file)
# Note: You'll need to implement file upload or use WebSocket
```

**Test LLM Service:**
```bash
# Health check
curl http://localhost:8002/health

# Test chat
curl -X POST http://localhost:8002/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello, what can you do?", "user_id": "test"}'

# Expected: JSON response with AI-generated text
```

**Test Vision Service:**
```bash
# Health check
curl http://localhost:8003/health

# Test detection (if you have a test image)
curl -X POST http://localhost:8003/detect \
  -F "image=@test_image.jpg"

# Expected: JSON with detections, person tracking, activity detection
```

### 4.4 Test ROS 2 Bridge (if ROS 2 installed)

```bash
cd ~/elderly_companion/ros2_workspace

# Build ROS 2 package
colcon build --packages-select companion_bridge

# Source workspace
source install/setup.bash

# Run bridge (in one terminal)
ros2 run companion_bridge ai_bridge

# In another terminal, check topics
source install/setup.bash
ros2 topic list
# Should see topics like:
# /ai/vision/detections
# /ai/speech/transcription
# /ai/llm/response

# Echo a topic to see data
ros2 topic echo /ai/vision/detections
```

### 4.5 Verify System Resources

```bash
# Check CPU and memory usage
htop
# Or
top

# Check GPU usage (if available)
sudo tegrastats

# Check disk space
df -h

# Check Docker resources
docker stats
```

### 4.6 Troubleshooting Tests

**If services don't start:**
```bash
# Check Python version
python3.11 --version

# Check virtual environment
which python
# Should point to venv/bin/python

# Check installed packages
pip list | grep -E "torch|whisper|ultralytics|fastapi"

# Check service logs
docker compose logs [service_name]
```

**If health checks fail:**
```bash
# Check if ports are in use
sudo netstat -tulpn | grep -E "8001|8002|8003"

# Check service logs
docker compose logs -f [service_name]

# Restart services
docker compose restart
```

**If Ollama connection fails (timeout or "falling back to rule-based"):**
```bash
# 1. Check if Ollama is installed
ollama --version
# If not installed, see Step 3.3 above

# 2. Check if Ollama service is running
systemctl status ollama
# OR check if process is running
ps aux | grep ollama

# 3. Start Ollama if not running
sudo systemctl start ollama
# OR run manually: ollama serve

# 4. Wait for Ollama to start (may take 5-10 seconds)
sleep 5

# 5. Verify Ollama is accessible
curl http://localhost:11434/api/tags
# Should return JSON with models, not an error

# 6. Check if model is downloaded
ollama list
# Should show at least one model (e.g., llama3.2:1b)

# 7. If model is missing, download it
ollama pull llama3.2:1b
# This may take several minutes depending on internet speed

# 8. Test Ollama directly
ollama run llama3.2:1b "Hello"
# Should return a response

# 9. Restart LLM service after Ollama is running
# Stop the service (Ctrl+C) and restart:
python scripts/llm_service_ollama.py
```

**Note:** The LLM service will work with rule-based fallback even if Ollama is not available. This is intentional for testing. However, for natural AI conversations, you need Ollama running and a model downloaded.

### 4.7 Success Criteria

**Before proceeding to hardware assembly, verify:**

- [ ] All three services start without errors
- [ ] Health endpoints return 200 OK
- [ ] Speech service loads Whisper model successfully
- [ ] LLM service connects to Ollama
- [ ] Vision service loads YOLO model successfully
- [ ] Docker services run and stay healthy
- [ ] ROS 2 bridge works (if ROS 2 installed)
- [ ] System resources are adequate (check memory/CPU)
- [ ] No critical errors in logs

**If all checks pass, you're ready for hardware assembly!**

---

## Step 5: Hardware Assembly (30-60 minutes)

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

## Step 6: Configure Services for Jetson (15 minutes)

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

## Step 7: Test Hardware Integration (After Assembly)

**After hardware assembly (Step 5), test hardware components:**

### 7.1 Test Camera

```bash
# Check if camera is detected
ls /dev/video*

# Install v4l2 tools if not already installed
sudo apt install -y v4l-utils

# List camera devices
v4l2-ctl --list-devices

# Test camera capture
fswebcam test.jpg
# Or use OpenCV
python3 -c "import cv2; cap = cv2.VideoCapture(0); ret, frame = cap.read(); cv2.imwrite('test.jpg', frame) if ret else print('Camera not working')"
```

### 7.2 Test Audio (Microphone/Speaker)

```bash
# List recording devices
arecord -l

# List playback devices
aplay -l

# Test microphone recording
arecord -d 3 -f cd test.wav

# Test speaker playback
aplay test.wav
```

### 7.3 Test Motors (if applicable)

```bash
# Test motor control (implementation depends on your motor driver)
# Example for GPIO-based control:
python3 << EOF
import Jetson.GPIO as GPIO
# Your motor test code here
EOF
```

### 7.4 Test Vision Service with Real Camera

```bash
# Update vision service to use camera device
# Then test with real camera feed
curl -X POST http://localhost:8003/detect \
  -F "image=@test.jpg"
```

---

## Step 8: Performance Optimization (Optional)

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

## Step 9: Deployment Checklist

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

