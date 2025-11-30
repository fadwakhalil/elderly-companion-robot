# Complete Setup Guide - Elderly Companion Robot

**This is your ONE comprehensive guide. Follow it from start to finish to set up the entire system.**

---

## 📋 Table of Contents

1. [Prerequisites](#prerequisites)
2. [Development Environment Setup](#development-environment-setup)
3. [Project Structure](#project-structure)
4. [Virtual Environment](#virtual-environment)
5. [AI Services Setup](#ai-services-setup)
6. [Ollama Integration](#ollama-integration)
7. [ROS 2 Integration](#ros-2-integration)
8. [Docker Setup](#docker-setup)
9. [Testing](#testing)
10. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### System Requirements

✅ **Your System:**
- macOS 15.7.1 (Sequoia) ✅
- Apple Silicon (arm64) ✅
- 243GB available space ✅

### Required Software

- Homebrew package manager
- Python 3.11+ (⚠️ **Use 3.11, not 3.13**)
- Docker Desktop
- ROS 2 Humble (optional, for robot control)

---

## Development Environment Setup

### Step 1: Install Homebrew (if not installed)

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zshrc
source ~/.zshrc
```

### Step 2: Install Core Tools

```bash
brew install python@3.11 cmake git wget curl pkg-config
python3.11 -m pip install --upgrade pip setuptools wheel
```

### Step 3: Install Docker Desktop

```bash
brew install --cask docker
open -a Docker
# Wait for Docker to start, then verify:
docker --version
docker compose version
```

### Step 4: Install ROS 2 Humble (Optional - for robot control)

```bash
brew install asio tinyxml2 opencv ffmpeg
sudo pip3 install -U rosdep
sudo rosdep init && rosdep update
brew install ros-humble-ros-base
echo "source /opt/homebrew/opt/ros/humble/setup.zsh" >> ~/.zshrc
source ~/.zshrc
ros2 --help  # Verify installation
```

---

## Project Structure

### Create Directory Structure

```bash
cd ~/Documents/ops/ai-project
mkdir -p elderly_companion/ai_services/{docker,config,models,scripts,test_data,logs}
mkdir -p elderly_companion/ros2_workspace/src
```

### Final Structure

```
~/Documents/ops/ai-project/
├── elderly_companion/
│   ├── ai_services/
│   │   ├── docker/              # Dockerfiles
│   │   ├── config/              # Service configurations
│   │   ├── models/              # ML models (YOLO, etc.)
│   │   ├── scripts/             # Service scripts
│   │   ├── test_data/           # Test files
│   │   └── logs/                # Service logs
│   └── ros2_workspace/
│       └── src/                 # ROS 2 packages
└── scripts/                     # Helper scripts
    ├── setup_venvs.sh
    └── activate_venv.sh
```

---

## Virtual Environment

### ⚠️ Important: Use Python 3.11

Many packages (torch, ultralytics, whisper) don't fully support Python 3.13 yet.

### Create Virtual Environment

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services

# Use Python 3.11 explicitly
python3.11 -m venv venv

# Activate
source venv/bin/activate

# Verify Python version
python --version  # Should show 3.11.x
```

**Or use helper script:**

```bash
cd ~/Documents/ops/ai-project
./scripts/setup_venvs.sh create_single
source scripts/activate_venv.sh all
```

---

## AI Services Setup

### Step 1: Create Requirements Files

#### Speech Service Requirements

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services

cat > requirements_speech.txt << 'EOF'
fastapi==0.104.1
uvicorn[standard]==0.24.0
pydantic==2.5.0
python-multipart==0.0.6
openai-whisper==20231117
torch==2.1.0
torchaudio==2.1.0
numpy==1.24.3
# rclpy==3.3.9  # Install via rosdep or ROS 2
EOF
```

#### LLM Service Requirements

```bash
cat > requirements_llm.txt << 'EOF'
fastapi==0.104.1
uvicorn[standard]==0.24.0
pydantic==2.5.0
python-multipart==0.0.6
pyyaml==6.0.1
httpx==0.25.2
# rclpy==3.3.9  # Install via rosdep or ROS 2
EOF
```

#### Vision Service Requirements

```bash
cat > requirements_vision.txt << 'EOF'
fastapi==0.104.1
uvicorn[standard]==0.24.0
pydantic==2.5.0
python-multipart==0.0.6
opencv-python==4.8.1.78
ultralytics==8.1.0
torch==2.1.0
torchvision==0.16.0
pillow==10.1.0
numpy==1.24.3
aiofiles==23.2.1
# rclpy==3.3.9  # Install via rosdep or ROS 2
EOF
```

### Step 2: Install Dependencies

```bash
# Make sure venv is activated
source venv/bin/activate

# Install all requirements
pip install --upgrade pip setuptools wheel
pip install -r requirements_speech.txt
pip install -r requirements_llm.txt
pip install -r requirements_vision.txt
```

### Step 3: Create Service Scripts

The service scripts are already created:
- `scripts/speech_service.py` - Speech-to-Text service
- `scripts/llm_service.py` - LLM service (rule-based)
- `scripts/llm_service_ollama.py` - LLM service with Ollama
- `scripts/vision_service.py` - Basic vision service
- `scripts/vision_service_enhanced.py` - Enhanced vision with tracking

### Step 4: Create Configuration Files

#### Personality Configuration

```bash
mkdir -p config

cat > config/personality.yaml << 'EOF'
name: "Companion"
traits:
  - empathetic
  - patient
  - supportive
  - proactive
conversation_style: "warm and caring"
proactive_interactions:
  - morning_greeting: "Good morning! How did you sleep?"
  - medication_reminder: "It's time for your medication."
  - activity_check: "Would you like to take a walk today?"
EOF
```

---

## Ollama Integration

### Step 1: Install Ollama

```bash
brew install ollama
```

### Step 2: Start Ollama Service

```bash
# Start as background service
brew services start ollama

# Or run manually
ollama serve
```

### Step 3: Download Model

```bash
# Download recommended model (3B parameters, good balance)
ollama pull llama3.2:3b

# Or smaller model for Jetson (1B parameters)
ollama pull llama3.2:1b
```

### Step 4: Test Ollama

```bash
ollama run llama3.2:3b "Hello, how are you?"
```

### Step 5: Use Enhanced LLM Service

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services
source venv/bin/activate

# Install httpx if not already installed
pip install httpx

# Run enhanced LLM service
python3 scripts/llm_service_ollama.py
```

**Test in another terminal:**

```bash
curl -X POST http://localhost:8002/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello, I feel lonely today", "user_id": "test1"}' | python3 -m json.tool
```

---

## ROS 2 Integration

### Step 1: Create ROS 2 Package

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ros2_workspace/src

# Create package (if not already created)
ros2 pkg create --build-type ament_python companion_bridge \
    --dependencies rclpy std_msgs geometry_msgs sensor_msgs
```

**Note:** The package structure is already created at:
- `ros2_workspace/src/companion_bridge/`

### Step 2: Build Package

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ros2_workspace
colcon build --packages-select companion_bridge
source install/setup.zsh
```

### Step 3: Run ROS 2 Nodes

```bash
# AI Bridge node
ros2 run companion_bridge ai_bridge

# Behavior Manager node
ros2 run companion_bridge behavior_manager
```

### ROS 2 Topics

**Subscribed Topics:**
- `/companion/speech/transcription` - Speech transcriptions
- `/companion/vision/detections` - Vision detections
- `/companion/llm/response` - LLM responses

**Published Topics:**
- `/companion/llm/request` - LLM requests
- `/companion/state` - Robot state
- `/companion/alert` - Emergency alerts
- `/cmd_vel` - Robot velocity commands
- `/goal_pose` - Navigation goals

---

## Docker Setup

### Step 1: Create Dockerfiles

Dockerfiles are already created in `docker/`:
- `Dockerfile.speech`
- `Dockerfile.llm`
- `Dockerfile.vision`

### Step 2: Create Docker Compose File

The `docker-compose.yml` is already created with:
- Speech service (port 8001)
- LLM service (port 8002)
- Vision service (port 8003)
- Resource limits
- Health checks

### Step 3: Build and Start Services

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services

# Build images
docker compose build

# Start services
docker compose up -d

# Check status
docker compose ps

# View logs
docker compose logs -f
```

### Step 4: Stop Services

```bash
docker compose down
```

---

## Testing

### Test Services Locally (Recommended First)

#### Test Speech Service

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services
source venv/bin/activate

# Run service
python3 scripts/speech_service.py

# In another terminal, test
curl http://localhost:8001/health
```

#### Test LLM Service (with Ollama)

```bash
# Make sure Ollama is running
brew services start ollama

# Run service
python3 scripts/llm_service_ollama.py

# Test
curl -X POST http://localhost:8002/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello", "user_id": "test"}' | python3 -m json.tool
```

#### Test Vision Service (Enhanced)

```bash
# Run service
python3 scripts/vision_service_enhanced.py

# Test with image
curl -X POST http://localhost:8003/detect \
  -F "file=@test_image.jpg" | python3 -m json.tool
```

### Test Docker Services

```bash
# Health checks
curl http://localhost:8001/health  # Speech
curl http://localhost:8002/health  # LLM
curl http://localhost:8003/health  # Vision

# Test LLM
curl -X POST http://localhost:8002/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello", "user_id": "test"}'

# Test Vision
curl -X POST http://localhost:8003/detect \
  -F "file=@test_image.jpg"
```

### Test ROS 2 Integration

```bash
# Terminal 1: Run AI bridge
ros2 run companion_bridge ai_bridge

# Terminal 2: Publish test message
ros2 topic pub /companion/speech/transcription std_msgs/String "data: 'Hello robot'"

# Terminal 3: Monitor topics
ros2 topic list
ros2 topic echo /companion/state
```

---

## Troubleshooting

### Python 3.13 Compatibility Issues

**Problem:** `No matching distribution found for torch==2.1.0` or similar errors

**Solution:** Use Python 3.11

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services
rm -rf venv
python3.11 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements_speech.txt
pip install -r requirements_llm.txt
pip install -r requirements_vision.txt
```

### Docker Build Errors

**Problem:** SSL errors during `torch` download

**Solution:** Retry build (network issue)

```bash
docker compose build --no-cache
```

**Problem:** `libGL.so.1` error in vision service

**Solution:** Already fixed in Dockerfile.vision (libgl1 installed)

### ROS 2 Not Found

**Problem:** `ros2: command not found`

**Solution:**

```bash
source /opt/homebrew/opt/ros/humble/setup.zsh
echo "source /opt/homebrew/opt/ros/humble/setup.zsh" >> ~/.zshrc
```

### Port Conflicts

**Problem:** Port already in use

**Solution:**

```bash
# Check what's using the port
lsof -i :8001
lsof -i :8002
lsof -i :8003

# Kill process or change port in docker-compose.yml
```

### Ollama Not Responding

**Problem:** Ollama service not running

**Solution:**

```bash
# Check status
brew services list | grep ollama

# Start service
brew services start ollama

# Test
curl http://localhost:11434/api/tags
```

### Vision Service Segmentation Fault (Docker)

**Problem:** Vision service crashes in Docker but works locally

**Solution:** 
- Use local execution for development
- Docker issues are known and being investigated
- See `elderly_companion/ai_services/LOCAL_EXECUTION_GUIDE.md` for local testing

### Module Not Found Errors

**Problem:** `ModuleNotFoundError: No module named 'X'`

**Solution:**

```bash
# Make sure venv is activated
source venv/bin/activate

# Install missing module
pip install X

# Or reinstall all requirements
pip install -r requirements_speech.txt
pip install -r requirements_llm.txt
pip install -r requirements_vision.txt
```

---

## Quick Reference

### Navigation

```bash
# Set aliases (add to ~/.zshrc)
export COMPANION_DIR="$HOME/Documents/ops/ai-project/elderly_companion"
export COMPANION_AI_DIR="$COMPANION_DIR/ai_services"
export COMPANION_WS_DIR="$COMPANION_DIR/ros2_workspace"

alias cd-companion="cd $COMPANION_DIR"
alias cd-ai="cd $COMPANION_AI_DIR"
alias cd-ws="cd $COMPANION_WS_DIR"
```

### Common Commands

```bash
# Activate virtual environment
cd ~/Documents/ops/ai-project/elderly_companion/ai_services
source venv/bin/activate

# Start Docker services
cd-ai && docker compose up -d

# Check service status
docker compose ps
docker compose logs -f [service_name]

# Build ROS 2 package
cd-ws
colcon build --packages-select companion_bridge
source install/setup.zsh

# Run ROS 2 nodes
ros2 run companion_bridge ai_bridge
ros2 run companion_bridge behavior_manager
```

### Service URLs

- Speech Service: http://localhost:8001
- LLM Service: http://localhost:8002
- Vision Service: http://localhost:8003
- Ollama API: http://localhost:11434

---

## Next Steps

After completing setup:

1. ✅ Test all services locally
2. ✅ Test Docker services
3. ✅ Integrate Ollama for natural conversations
4. ✅ Test ROS 2 bridge (if ROS 2 installed)
5. ✅ Test enhanced vision with person tracking
6. ✅ Build and test on hardware (when available)

---

## Additional Resources

### Service-Specific Guides

- **Local Execution:** `elderly_companion/ai_services/LOCAL_EXECUTION_GUIDE.md`
- **Docker Resources:** `elderly_companion/ai_services/DOCKER_RESOURCES.md`
- **Vision Troubleshooting:** `elderly_companion/ai_services/VISION_SERVICE_TROUBLESHOOTING.md`

### Project Files

- **Service Scripts:** `elderly_companion/ai_services/scripts/`
- **Dockerfiles:** `elderly_companion/ai_services/docker/`
- **ROS 2 Package:** `elderly_companion/ros2_workspace/src/companion_bridge/`

---

**This guide contains everything you need to set up the system from scratch. Follow it step by step, and you'll have a fully functional AI companion robot system!** 🚀

