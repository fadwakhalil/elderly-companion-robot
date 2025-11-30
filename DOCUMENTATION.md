# Documentation Index

**Complete documentation for the Elderly Companion Robot project.**

---

## 🚀 Getting Started

### **[README.md](README.md)** - Start Here
- Project overview
- Quick start guide
- Links to all documentation

### **[NEXT_STEPS.md](NEXT_STEPS.md)** ⭐ - What to Do Next
- Complete action plan
- Step-by-step verification
- Testing procedures
- Development priorities

---

## 📚 Main Guides

### **[COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md)** ⭐ - Development Environment Setup
**For setting up your Mac development environment**
- Prerequisites and system requirements
- Development environment setup
- Virtual environment configuration
- AI services setup
- Ollama integration
- ROS 2 integration
- Docker setup
- Testing procedures
- Troubleshooting

### **[HARDWARE_SETUP_GUIDE.md](HARDWARE_SETUP_GUIDE.md)** ⭐ - Jetson Hardware Setup
**For setting up and deploying to your Jetson**
- Post-flash configuration
- System updates and dependencies
- Python environment setup
- Docker installation
- ROS 2 installation
- Project deployment
- Hardware assembly
- Performance optimization
- Testing and verification

---

## 🔧 Service-Specific Guides

### AI Services Documentation

Located in `elderly_companion/ai_services/`:

- **[LOCAL_EXECUTION_GUIDE.md](elderly_companion/ai_services/LOCAL_EXECUTION_GUIDE.md)** - Running services locally (outside Docker)
- **[DOCKER_RESOURCES.md](elderly_companion/ai_services/DOCKER_RESOURCES.md)** - Docker configuration and resource limits
- **[VISION_TROUBLESHOOTING.md](elderly_companion/ai_services/VISION_TROUBLESHOOTING.md)** - Vision service troubleshooting

---

## 📁 Project Structure

```
~/Documents/ops/ai-project/
├── README.md                        # Project overview
├── DOCUMENTATION.md                 # This file
├── COMPLETE_SETUP_GUIDE.md         # Mac development setup
├── HARDWARE_SETUP_GUIDE.md         # Jetson deployment
├── docs/
│   └── archive/                     # Archived flashing guides
├── elderly_companion/
│   ├── ai_services/                 # AI Services
│   │   ├── LOCAL_EXECUTION_GUIDE.md
│   │   ├── DOCKER_RESOURCES.md
│   │   ├── VISION_TROUBLESHOOTING.md
│   │   └── [service files]
│   └── ros2_workspace/              # ROS 2 Workspace
│       └── src/
│           └── companion_bridge/
└── scripts/                         # Helper scripts
```

---

## 🎯 Quick Navigation

**I want to...**

- **Set up my Mac for development** → [COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md)
- **Deploy to my Jetson** → [HARDWARE_SETUP_GUIDE.md](HARDWARE_SETUP_GUIDE.md)
- **Run services locally** → [LOCAL_EXECUTION_GUIDE.md](elderly_companion/ai_services/LOCAL_EXECUTION_GUIDE.md)
- **Configure Docker** → [DOCKER_RESOURCES.md](elderly_companion/ai_services/DOCKER_RESOURCES.md)
- **Troubleshoot vision service** → [VISION_TROUBLESHOOTING.md](elderly_companion/ai_services/VISION_TROUBLESHOOTING.md)
- **See archived flashing guides** → `docs/archive/` (for reference only)

---

## 📝 Documentation Status

✅ **Current:** All active documentation is up to date  
✅ **Organized:** Clear structure and navigation  
✅ **Centralized:** Single source of truth  
📦 **Archived:** Flashing guides moved to `docs/archive/` (for reference)

---

**Need help?** Start with [README.md](README.md) or the appropriate guide above!

