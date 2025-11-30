# Documentation Index

Complete list of all documentation files and their purposes.

---

## 📚 Main Documentation

### **[README.md](README.md)** - Project Overview
- Quick start guide
- Project overview
- Links to all documentation

### **[HARDWARE_SETUP_GUIDE.md](HARDWARE_SETUP_GUIDE.md)** ⭐ - **HARDWARE SETUP**
- **Follow this when hardware arrives!**
- Jetson flashing with JetPack
- Hardware assembly
- Environment setup on Jetson
- Service deployment
- Performance optimization
- Hardware testing

### **[SDK_MANAGER_WALKTHROUGH.md](SDK_MANAGER_WALKTHROUGH.md)** ⭐ - **SDK MANAGER DETAILED GUIDE**
- **Complete step-by-step walkthrough of SDK Manager**
- Step 01: Development Environment configuration
- Step 02: Details and License
- Step 03: Setup Process and Jetson connection
- Step 04: Summary Finalization
- Troubleshooting common issues
- Quick reference checklist

### **[COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md)** ⭐ - **DEVELOPMENT SETUP**
- **Follow this for Mac development environment**
- Prerequisites
- Development environment setup
- Project structure
- Virtual environment
- AI services setup
- Ollama integration
- ROS 2 integration
- Docker setup
- Testing procedures
- Troubleshooting

---

## 💻 VM Setup Guides (For Jetson Flashing)

### **[UTM_SETUP.md](UTM_SETUP.md)** - UTM VM Setup (Apple Silicon Mac)
- Step-by-step UTM VM creation
- Ubuntu installation
- USB passthrough configuration
- SDK Manager installation

### **[VIRTUALBOX_SETUP.md](VIRTUALBOX_SETUP.md)** - VirtualBox VM Setup (Intel Mac)
- Step-by-step VirtualBox VM creation
- Ubuntu installation
- USB 3.0 Extension Pack setup
- SDK Manager installation

### **[FRESH_VM_START.md](FRESH_VM_START.md)** 🔄 - Fresh VM Start
- Complete cleanup guide
- Delete existing VM completely
- Start fresh installation
- Avoid common mistakes

### **[BOOTABLE_USB_FLASH_METHOD.md](BOOTABLE_USB_FLASH_METHOD.md)** ⭐ - Bootable USB Flash Method
- **Best alternative to VM method!**
- Create bootable Ubuntu USB
- Flash Jetson directly (no VM USB issues)
- Complete step-by-step guide

---

## 🔧 Service-Specific Guides

### `elderly_companion/ai_services/`

#### **[LOCAL_EXECUTION_GUIDE.md](elderly_companion/ai_services/LOCAL_EXECUTION_GUIDE.md)**
- How to run services locally (outside Docker)
- Local testing procedures
- Development workflow

#### **[DOCKER_RESOURCES.md](elderly_companion/ai_services/DOCKER_RESOURCES.md)**
- Docker configuration details
- Resource limits
- Docker optimization

#### **[VISION_TROUBLESHOOTING.md](elderly_companion/ai_services/VISION_TROUBLESHOOTING.md)**
- Vision service troubleshooting
- YOLO/PyTorch issues
- Docker segfault workarounds
- Performance optimization

---

## 📁 Project Structure

```
~/Documents/ops/ai-project/
├── README.md                        # Project overview
├── COMPLETE_SETUP_GUIDE.md         # ⭐ Main setup guide
├── DOCUMENTATION_INDEX.md          # This file
├── elderly_companion/
│   ├── ai_services/
│   │   ├── LOCAL_EXECUTION_GUIDE.md
│   │   ├── DOCKER_RESOURCES.md
│   │   ├── VISION_TROUBLESHOOTING.md
│   │   └── [service files]
│   └── ros2_workspace/
│       └── [ROS 2 packages]
└── scripts/
    └── [helper scripts]
```

---

## 🚀 Quick Start

1. **Read:** [README.md](README.md) for overview
2. **Follow:** [COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md) for setup
3. **Reference:** Service-specific guides as needed

---

## 📝 Documentation Status

✅ **Consolidated:** All setup steps in one guide
✅ **Cleaned:** Removed redundant files
✅ **Organized:** Clear structure and navigation
✅ **Complete:** Everything needed for setup and troubleshooting

---

**All documentation is now centralized and easy to navigate!**

