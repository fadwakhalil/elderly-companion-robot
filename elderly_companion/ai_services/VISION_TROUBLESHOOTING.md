# Vision Service Troubleshooting Guide

Complete troubleshooting guide for vision service issues, including YOLO/PyTorch configuration and Docker problems.

---

## Common Issues

### 1. Segmentation Fault in Docker (Exit Code 139)

**Problem:** Vision service crashes with segmentation fault during YOLO inference in Docker, but works locally.

**Status:** Known issue - Docker-specific segfault persists despite extensive configuration.

**Workaround:** Use local execution for development and testing.

**Solutions Attempted:**
- ✅ CPU-only PyTorch installation
- ✅ Explicit CPU device configuration
- ✅ Thread limits (OMP_NUM_THREADS, MKL_NUM_THREADS)
- ✅ Image resizing to 640x640
- ✅ Safer tensor-to-numpy conversions
- ✅ Resource limits in docker-compose.yml

**Current Configuration:**
- PyTorch: 2.1.0+cpu
- Ultralytics: 8.1.0
- Device: CPU-only
- Image size: 640x640 max

**Local Execution (Recommended):**
```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services
source venv/bin/activate
python3 scripts/vision_service_enhanced.py
```

See `LOCAL_EXECUTION_GUIDE.md` for complete local testing instructions.

---

### 2. libGL.so.1 Error

**Problem:** `ImportError: libGL.so.1: cannot open shared object file`

**Solution:** Already fixed in Dockerfile.vision - `libgl1` package is installed.

If you see this error, rebuild the Docker image:
```bash
docker compose build --no-cache vision-service
```

---

### 3. Module Not Found: rclpy

**Problem:** `ModuleNotFoundError: No module named 'rclpy'`

**Solution:** rclpy is optional. The service will run without ROS 2 integration.

If you need ROS 2:
```bash
# Install ROS 2 first
brew install ros-humble-ros-base
source /opt/homebrew/opt/ros/humble/setup.zsh

# Then rclpy will be available
```

---

### 4. YOLO Model Download Issues

**Problem:** Model download fails or is slow

**Solution:** 
- Model is cached after first download
- Check internet connection
- Model downloads to `models/` directory

**Manual Download:**
```bash
# Download model manually
cd ~/Documents/ops/ai-project/elderly_companion/ai_services/models
wget https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8n.pt
```

---

### 5. Memory Issues

**Problem:** Out of memory errors during inference

**Solutions:**
- Use smaller model: `yolov8n.pt` (nano) instead of larger models
- Resize images before processing (already implemented)
- Reduce batch size (currently 1)
- Increase Docker memory limit in docker-compose.yml

**Docker Memory:**
```yaml
# In docker-compose.yml
vision-service:
  deploy:
    resources:
      limits:
        memory: 4G  # Increase if needed
```

---

### 6. Slow Inference

**Problem:** Detection is slow

**Solutions:**
- Use smaller model (yolov8n.pt)
- Reduce image size (already resized to 640x640 max)
- Use CPU-only (already configured)
- For faster inference, consider GPU support (requires NVIDIA GPU)

---

## Alternative Configurations

### Option 1: Newer Versions (Not Tested)

```dockerfile
# In Dockerfile.vision
RUN pip install --no-cache-dir \
    torch==2.2.0 \
    torchvision==0.17.0 \
    ultralytics==8.2.0
```

### Option 2: Older Versions (Not Tested)

```dockerfile
# In Dockerfile.vision
RUN pip install --no-cache-dir \
    torch==2.0.1 \
    torchvision==0.15.2 \
    ultralytics==8.0.200
```

**Note:** These are untested. Current configuration (2.1.0) is recommended.

---

## Testing

### Test Locally

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services
source venv/bin/activate
python3 scripts/vision_service_enhanced.py

# In another terminal
curl -X POST http://localhost:8003/detect \
  -F "file=@test_image.jpg" | python3 -m json.tool
```

### Test Docker

```bash
docker compose up -d vision-service
docker compose logs -f vision-service

# Test
curl -X POST http://localhost:8003/detect \
  -F "file=@test_image.jpg"
```

---

## Performance Optimization

### For Jetson Hardware

When deploying to Jetson:
1. Use TensorRT for faster inference
2. Use smaller model (yolov8n.pt)
3. Consider quantization
4. Optimize image preprocessing

### Current Optimizations

- ✅ Image resizing to 640x640 max
- ✅ CPU-only inference
- ✅ Single image processing (no batching)
- ✅ Efficient tensor handling

---

## Debugging

### Enable Debug Logging

```python
# In vision_service_enhanced.py
logging.basicConfig(level=logging.DEBUG)
```

### Check Model Loading

```bash
# Test model loading
python3 -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt'); print('Model loaded')"
```

### Check Tensor Operations

Add debug prints in `detect_objects()` method to see tensor shapes and types.

---

## Resources

- **Local Execution Guide:** `LOCAL_EXECUTION_GUIDE.md`
- **Docker Resources:** `DOCKER_RESOURCES.md`
- **YOLO Documentation:** https://docs.ultralytics.com/
- **PyTorch CPU:** https://pytorch.org/get-started/locally/

---

**For most issues, use local execution for development. Docker issues are known and being investigated.**

