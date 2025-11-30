# Next Steps - Action Plan

**Your Jetson is set up and accessible. Here's what to do next!**

---

## 🎯 Current Status

✅ Jetson Nano is flashed and accessible  
✅ Project structure is in place  
✅ AI services code is ready  
✅ Docker configuration is ready  
✅ Virtual environment exists  

---

## 📋 Step-by-Step Action Plan

### Phase 1: Verify Mac Development Environment (15-30 minutes)

**Goal:** Ensure your Mac is ready for development and testing.

#### 1.1 Check Prerequisites

```bash
# Check Python version (should be 3.11)
python3.11 --version

# Check Docker
docker --version
docker compose version

# Check if Docker is running
docker ps
```

**If anything is missing:**
- Follow [COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md) to install prerequisites

#### 1.2 Verify Virtual Environment

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services

# Activate virtual environment
source venv/bin/activate

# Check Python version in venv
python --version  # Should be 3.11.x

# Check if key packages are installed
pip list | grep -E "torch|whisper|ultralytics|fastapi|uvicorn"
```

**If packages are missing:**
```bash
# Install requirements
pip install -r requirements_speech.txt
pip install -r requirements_llm.txt
pip install -r requirements_vision.txt
```

#### 1.3 Install Ollama (for LLM service)

```bash
# Install Ollama on Mac
brew install ollama

# Start Ollama
ollama serve

# In another terminal, download model
ollama pull llama3.2:1b
```

---

### Phase 2: Test Services Locally on Mac (30-45 minutes)

**Goal:** Verify all services work on your Mac before deploying to Jetson.

#### 2.1 Test Services Individually

**Terminal 1 - Speech Service:**
```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services
source venv/bin/activate
python scripts/speech_service.py
```

**Terminal 2 - LLM Service:**
```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services
source venv/bin/activate
python scripts/llm_service_ollama.py
```

**Terminal 3 - Vision Service:**
```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services
source venv/bin/activate
python scripts/vision_service_enhanced.py
```

**Test each service:**
```bash
# Test Speech Service
curl http://localhost:8001/health

# Test LLM Service
curl http://localhost:8002/health
curl -X POST http://localhost:8002/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello", "user_id": "test"}'

# Test Vision Service
curl http://localhost:8003/health
curl -X POST http://localhost:8003/detect \
  -F "image=@test_image.jpg"
```

#### 2.2 Test with Docker

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services

# Build images
docker compose build

# Start all services
docker compose up -d

# Check status
docker compose ps

# View logs
docker compose logs -f

# Test services (same as above)
curl http://localhost:8001/health
curl http://localhost:8002/health
curl http://localhost:8003/health

# Stop services when done
docker compose down
```

**✅ Success Criteria:**
- All three services start without errors
- Health endpoints return 200 OK
- Services can process requests

---

### Phase 3: Deploy to Jetson (45-60 minutes)

**Goal:** Get your services running on the Jetson hardware.

#### 3.1 Transfer Project to Jetson

**Option A: Git Clone (Recommended)**
```bash
# On Jetson
cd ~
git clone <your-repo-url> ai-project
cd ~/ai-project/elderly_companion
```

**Option B: SCP from Mac**
```bash
# On Mac
cd ~/Documents/ops/ai-project
scp -r elderly_companion jetson@<jetson-ip>:~/
```

#### 3.2 Set Up Jetson Environment

**SSH to Jetson:**
```bash
ssh jetson@<jetson-ip>
```

**Follow [HARDWARE_SETUP_GUIDE.md](HARDWARE_SETUP_GUIDE.md) Step 1-3:**
- Update system
- Install Python 3.11
- Install Docker
- Create virtual environment
- Install dependencies
- Install Ollama

#### 3.3 Test Services on Jetson

```bash
# On Jetson
cd ~/elderly_companion/ai_services

# Test locally first
source venv/bin/activate
python scripts/speech_service.py  # Test in one terminal

# Then test with Docker
docker compose build
docker compose up -d
docker compose ps
docker compose logs -f
```

**✅ Success Criteria:**
- Services build successfully
- Services start and stay running
- Health endpoints respond

---

### Phase 4: Integration Testing (30-45 minutes)

**Goal:** Verify services work together and with ROS 2.

#### 4.1 Test Service Integration

**Test end-to-end flow:**
```bash
# 1. Start all services
docker compose up -d

# 2. Test speech → LLM flow
# (You'll need to implement this or test manually)

# 3. Test vision → alerts flow
# (You'll need to implement this or test manually)
```

#### 4.2 Test ROS 2 Bridge (if ROS 2 installed)

```bash
# On Jetson
cd ~/elderly_companion/ros2_workspace

# Build ROS 2 package
colcon build --packages-select companion_bridge

# Source workspace
source install/setup.bash

# Run bridge
ros2 run companion_bridge ai_bridge

# In another terminal, check topics
ros2 topic list
ros2 topic echo /ai/vision/detections
```

---

### Phase 5: Development Priorities

**Now that everything is working, here's what to focus on:**

#### 5.1 Immediate Priorities

1. **Hardware Integration**
   - [ ] Connect and test camera
   - [ ] Connect and test microphone
   - [ ] Connect and test speakers
   - [ ] Test motor control (if applicable)

2. **Service Improvements**
   - [ ] Optimize vision service for Jetson GPU
   - [ ] Fine-tune LLM responses
   - [ ] Improve speech recognition accuracy
   - [ ] Add error handling and retry logic

3. **ROS 2 Integration**
   - [ ] Complete AI bridge implementation
   - [ ] Implement behavior manager
   - [ ] Add navigation integration
   - [ ] Test robot control commands

#### 5.2 Feature Development

1. **Conversation Features**
   - [ ] Add conversation history
   - [ ] Implement context awareness
   - [ ] Add personality customization
   - [ ] Create conversation flows

2. **Safety Features**
   - [ ] Improve fall detection accuracy
   - [ ] Add alert system
   - [ ] Implement emergency protocols
   - [ ] Add activity monitoring

3. **Robot Behavior**
   - [ ] Implement autonomous navigation
   - [ ] Add person following
   - [ ] Create interaction behaviors
   - [ ] Add gesture recognition

#### 5.3 Production Readiness

1. **Reliability**
   - [ ] Add comprehensive error handling
   - [ ] Implement service health monitoring
   - [ ] Add automatic recovery
   - [ ] Create backup/restore procedures

2. **Performance**
   - [ ] Optimize for Jetson resources
   - [ ] Implement caching strategies
   - [ ] Add load balancing (if needed)
   - [ ] Profile and optimize bottlenecks

3. **Deployment**
   - [ ] Create deployment scripts
   - [ ] Add systemd services
   - [ ] Implement auto-start on boot
   - [ ] Create update procedures

---

## 🚀 Quick Start Commands

### Mac Development

```bash
# Navigate to project
cd ~/Documents/ops/ai-project/elderly_companion/ai_services

# Activate environment
source venv/bin/activate

# Start services
docker compose up -d

# View logs
docker compose logs -f [service_name]

# Stop services
docker compose down
```

### Jetson Deployment

```bash
# SSH to Jetson
ssh jetson@<jetson-ip>

# Navigate to project
cd ~/elderly_companion/ai_services

# Start services
docker compose up -d

# Check status
docker compose ps
docker compose logs -f
```

### Git Workflow

```bash
# Navigate to project root
cd ~/Documents/ops/ai-project

# Check status
git status

# Add changes
git add .

# Commit
git commit -m "Description of changes"

# Push to remote
git push

# Pull latest changes
git pull
```

---

## Phase 6: Git Workflow and Version Control

**Goal:** Set up version control and push your project to Git.

### 6.1 Initialize Git Repository (if not already done)

```bash
# Navigate to project root
cd ~/Documents/ops/ai-project

# Check if git is already initialized
git status

# If not initialized, initialize it
git init
```

### 6.2 Create .gitignore File

**Create a `.gitignore` file in the project root:**

```bash
cd ~/Documents/ops/ai-project
cat > .gitignore << 'EOF'
# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
venv/
env/
ENV/
.venv

# Virtual environments
elderly_companion/ai_services/venv/
scripts/venv/

# IDE
.vscode/
.idea/
*.swp
*.swo
*~

# OS
.DS_Store
Thumbs.db

# Logs
*.log
logs/
elderly_companion/ai_services/logs/

# Models (large files - consider Git LFS)
*.pt
*.pth
*.onnx
*.h5
*.pb
models/
elderly_companion/ai_services/models/*.pt
elderly_companion/ai_services/models/*.pth

# Docker
.dockerignore

# Environment variables
.env
.env.local

# Test data (optional - remove if you want to track test files)
test_data/
*.jpg
*.png
*.jpeg
!elderly_companion/ai_services/test_data/.gitkeep

# Build artifacts
build/
dist/
*.egg-info/

# ROS 2
install/
log/
build/
elderly_companion/ros2_workspace/install/
elderly_companion/ros2_workspace/log/
elderly_companion/ros2_workspace/build/
EOF
```

### 6.3 Add Files to Git

```bash
# Check what will be added
git status

# Add all files (respects .gitignore)
git add .

# Or add specific files/directories
git add elderly_companion/
git add *.md
git add .gitignore
```

### 6.4 Make Your First Commit

```bash
# Commit with a descriptive message
git commit -m "Initial commit: Elderly Companion Robot project

- AI services (Speech, LLM, Vision)
- Docker configuration
- ROS 2 integration
- Documentation
- Fixed indentation errors in service scripts"
```

### 6.5 Set Up Remote Repository

**Option A: Create New Repository on GitHub/GitLab**

1. **Create a new repository on GitHub/GitLab:**
   - Go to GitHub.com or GitLab.com
   - Click "New repository"
   - Name it (e.g., `elderly-companion-robot`)
   - **Don't** initialize with README, .gitignore, or license (you already have these)
   - Click "Create repository"

2. **Add remote and push:**
```bash
# Add remote (replace with your actual repository URL)
git remote add origin https://github.com/yourusername/elderly-companion-robot.git

# Or if using SSH:
# git remote add origin git@github.com:yourusername/elderly-companion-robot.git

# Verify remote
git remote -v

# Push to remote
git branch -M main  # Rename branch to 'main' if needed
git push -u origin main
```

**Option B: Use Existing Repository**

```bash
# If you already have a remote repository
git remote add origin <your-repo-url>

# Or update existing remote
git remote set-url origin <your-repo-url>

# Push
git push -u origin main
```

### 6.6 Regular Git Workflow

**For daily development:**

```bash
# 1. Check status
git status

# 2. Add changes
git add <file-or-directory>
# Or add all changes:
git add .

# 3. Commit with descriptive message
git commit -m "Description of changes

- What changed
- Why it changed
- Any important notes"

# 4. Push to remote
git push

# 5. If working with others, pull latest changes first
git pull
```

### 6.7 Useful Git Commands

```bash
# View commit history
git log --oneline

# View changes in a file
git diff <filename>

# View changes staged for commit
git diff --staged

# Undo changes to a file (before staging)
git checkout -- <filename>

# Unstage a file (keep changes)
git reset HEAD <filename>

# Create a new branch
git checkout -b feature/new-feature

# Switch branches
git checkout main

# Merge branch
git checkout main
git merge feature/new-feature

# View branches
git branch

# View remote branches
git branch -r
```

### 6.8 Best Practices

**Commit Messages:**
- Use clear, descriptive messages
- Start with a verb (Add, Fix, Update, Remove)
- Keep first line under 50 characters
- Add details in body if needed

**Example good commit messages:**
```
Fix: Correct indentation errors in speech_service.py

- Fixed ROS2_AVAILABLE conditional blocks
- Fixed super().__init__() indentation
- Service now starts without syntax errors
```

```
Add: Docker health checks for all services

- Added healthcheck endpoints
- Configured Docker health checks
- Improved service monitoring
```

**Branching Strategy:**
- `main` - Production-ready code
- `develop` - Development branch
- `feature/*` - New features
- `fix/*` - Bug fixes
- `docs/*` - Documentation updates

**Example:**
```bash
# Create feature branch
git checkout -b feature/add-fall-detection

# Make changes, commit
git add .
git commit -m "Add: Fall detection algorithm"

# Push feature branch
git push -u origin feature/add-fall-detection

# Create pull request on GitHub/GitLab
# After review, merge to main
```

### 6.9 Troubleshooting

**If you get "fatal: not a git repository":**
```bash
cd ~/Documents/ops/ai-project
git init
```

**If you get authentication errors:**
```bash
# For HTTPS, use personal access token instead of password
# For SSH, set up SSH keys:
ssh-keygen -t ed25519 -C "your_email@example.com"
# Then add public key to GitHub/GitLab
```

**If you need to update remote URL:**
```bash
git remote set-url origin <new-url>
```

**If you want to remove a file from Git but keep it locally:**
```bash
git rm --cached <filename>
# Then add to .gitignore
```

---

## 📚 Reference Guides

- **[COMPLETE_SETUP_GUIDE.md](COMPLETE_SETUP_GUIDE.md)** - Mac development setup
- **[HARDWARE_SETUP_GUIDE.md](HARDWARE_SETUP_GUIDE.md)** - Jetson deployment
- **[LOCAL_EXECUTION_GUIDE.md](elderly_companion/ai_services/LOCAL_EXECUTION_GUIDE.md)** - Local testing
- **[DOCKER_RESOURCES.md](elderly_companion/ai_services/DOCKER_RESOURCES.md)** - Docker configuration
- **[VISION_TROUBLESHOOTING.md](elderly_companion/ai_services/VISION_TROUBLESHOOTING.md)** - Vision service troubleshooting

---

## ✅ Checklist

**Before moving to development:**

- [ ] Mac development environment verified
- [ ] Services tested locally on Mac
- [ ] Project transferred to Jetson
- [ ] Jetson environment set up
- [ ] Services tested on Jetson
- [ ] Docker services working on Jetson
- [ ] ROS 2 bridge tested (if applicable)
- [ ] Hardware components connected
- [ ] Basic integration tested
- [ ] Git repository initialized
- [ ] .gitignore file created
- [ ] Initial commit made
- [ ] Remote repository set up
- [ ] Code pushed to remote

---

## 🎯 Recommended Next Action

**Start with Phase 1** - Verify your Mac development environment is ready, then test services locally before deploying to Jetson.

**Command to get started:**
```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services
source venv/bin/activate
python scripts/speech_service.py
```

**Then test the other services and move to Phase 2!**

---

**Need help?** Check the troubleshooting sections in the guides or review the service-specific documentation.

