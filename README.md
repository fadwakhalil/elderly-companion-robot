# Elderly Companion Robot Project

**AI-powered companion robot with ROS 2, Docker, and containerized AI services.**

---

## 🚀 Quick Start

### Your Jetson is Set Up? Great!

1. **Set up Mac development environment** → [COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md)
2. **Deploy to Jetson** → [HARDWARE_SETUP_GUIDE.md](HARDWARE_SETUP_GUIDE.md)
3. **Start developing!**

---

## 📋 Project Overview

### What This Project Is

An AI-powered companion robot system that provides:
- **Proactive Companionship** - Natural conversations with local LLM (Ollama)
- **Safety Monitoring** - Person tracking, activity detection, fall detection
- **Privacy by Design** - All AI processing happens locally
- **ROS 2 Integration** - Ready for robot control and navigation

### Architecture

- **AI Services** - Containerized services (Speech, LLM, Vision)
- **ROS 2 Bridge** - Connects AI services to robot control
- **Docker** - Containerized deployment
- **Jetson Orin Nano** - Edge AI hardware platform

---

## 📁 Project Structure

```
~/Documents/ops/ai-project/
├── README.md                        # This file
├── DOCUMENTATION.md                 # Documentation index
├── COMPLETE_SETUP_GUIDE.md         # Mac development setup
├── HARDWARE_SETUP_GUIDE.md         # Jetson deployment
├── elderly_companion/
│   ├── ai_services/                 # AI Services
│   │   ├── docker/                  # Dockerfiles
│   │   ├── config/                  # Service configurations
│   │   ├── models/                  # ML models
│   │   ├── scripts/                 # Service scripts
│   │   └── logs/                    # Service logs
│   └── ros2_workspace/              # ROS 2 Workspace
│       └── src/
│           └── companion_bridge/    # ROS 2 bridge package
└── scripts/                         # Helper scripts
```

---

## ⚡ Quick Commands

### Development (Mac)

```bash
# Navigate to project
cd ~/Documents/ops/ai-project

# Activate virtual environment
cd elderly_companion/ai_services
source venv/bin/activate

# Start Docker services
docker compose up -d

# Check service status
docker compose ps
docker compose logs -f [service_name]
```

### Deployment (Jetson)

```bash
# SSH to Jetson
ssh username@jetson-ip

# Navigate to project
cd ~/elderly_companion/ai_services

# Start services
docker compose up -d
```

---

## 🎯 Features

### AI Services

- ✅ **Speech-to-Text** - OpenAI Whisper (local)
- ✅ **LLM Service** - Ollama integration (Llama 3.2)
- ✅ **Vision Service** - YOLOv8 with person tracking and activity detection

### Enhanced Capabilities

- ✅ **Person Tracking** - Track same person across frames
- ✅ **Activity Detection** - Standing, sitting, lying
- ✅ **Fall Detection** - Automatic fall detection with alerts
- ✅ **Natural Conversations** - Empathetic, context-aware responses

### ROS 2 Integration

- ✅ **AI Bridge** - Connects AI services to robot control
- ✅ **Behavior Manager** - State machine for autonomous behaviors
- ✅ **Navigation Ready** - Prepared for robot navigation

---

## 📚 Documentation

### Main Guides

- **[DOCUMENTATION.md](DOCUMENTATION.md)** - Complete documentation index
- **[COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md)** ⭐ - Mac development environment setup
- **[HARDWARE_SETUP_GUIDE.md](HARDWARE_SETUP_GUIDE.md)** ⭐ - Jetson deployment and configuration

### Service-Specific Guides

- `elderly_companion/ai_services/LOCAL_EXECUTION_GUIDE.md` - Local testing guide
- `elderly_companion/ai_services/DOCKER_RESOURCES.md` - Docker configuration
- `elderly_companion/ai_services/VISION_TROUBLESHOOTING.md` - Vision service troubleshooting

---

## 🐳 Docker Services

### Service URLs (Local Development)

- Speech: http://localhost:8001
- LLM: http://localhost:8002
- Vision: http://localhost:8003

### Common Commands

```bash
# Start all services
docker compose up -d

# Stop all services
docker compose down

# View logs
docker compose logs -f [service_name]

# Rebuild services
docker compose build --no-cache
```

---

## 🔧 Prerequisites

### Mac Development

- macOS (tested on 15.7.1)
- Homebrew
- Python 3.11+ (⚠️ **Use 3.11, not 3.13**)
- Docker Desktop

### Jetson

- NVIDIA Jetson Orin Nano (8GB)
- JetPack 6.x
- Docker installed
- ROS 2 Humble (optional)

---

## 🐍 Virtual Environment

### Recommended: Single Virtual Environment

```bash
cd ~/Documents/ops/ai-project
./scripts/setup_venvs.sh create_single
source scripts/activate_venv.sh all
```

**See [COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md) for details.**

---

## 🆘 Troubleshooting

### Common Issues

**Python 3.13 compatibility:**
- Use Python 3.11 instead
- See [COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md) troubleshooting section

**Docker build errors:**
- Check Docker is running: `docker ps`
- Retry build: `docker compose build --no-cache`

**ROS 2 not found:**
```bash
source /opt/homebrew/opt/ros/humble/setup.zsh
```

**See [COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md) for complete troubleshooting guide.**

---

## 📖 Quick Reference

### Navigation Aliases

```bash
# Add to ~/.zshrc
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
cd-ai && source venv/bin/activate

# Start Docker services
cd-ai && docker compose up -d

# Build ROS 2 package
cd-ws
colcon build --packages-select companion_bridge
source install/setup.zsh
```

---

## 📝 Project Status

- ✅ AI Services (Speech, LLM, Vision) - Complete
- ✅ Ollama Integration - Complete
- ✅ Enhanced Vision (Tracking, Activity, Fall Detection) - Complete
- ✅ ROS 2 Bridge - Complete
- ✅ Docker Setup - Complete
- ✅ Documentation - Complete

---

## 🎓 Next Steps

**👉 Start here:** **[NEXT_STEPS.md](NEXT_STEPS.md)** - Complete action plan for what to do next!

1. ✅ Set up Mac development environment → [COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md)
2. ✅ Deploy to Jetson → [HARDWARE_SETUP_GUIDE.md](HARDWARE_SETUP_GUIDE.md)
3. ✅ Test all services → [NEXT_STEPS.md](NEXT_STEPS.md)
4. ✅ Start developing! → [NEXT_STEPS.md](NEXT_STEPS.md)

---

**Ready to start?** Follow **[COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md)** for Mac setup or **[HARDWARE_SETUP_GUIDE.md](HARDWARE_SETUP_GUIDE.md)** for Jetson deployment! 🚀
