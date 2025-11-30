# Running Vision Service Locally

The vision service has been updated to work both in Docker and locally. This guide explains how to run it locally to test if the segfault issue is Docker-specific.

## Changes Made

### ✅ Fixed Path Issues
- **Before:** Hardcoded `/app/models` path (Docker-only)
- **After:** Configurable paths using `APP_BASE_PATH` environment variable
- **Default:** Uses current working directory when running locally

### ✅ Fixed Port Configuration
- **Before:** Hardcoded port 8003
- **After:** Configurable via `VISION_SERVICE_PORT` environment variable
- **Default:** Port 8003

### ✅ Automatic Directory Creation
- Service now creates `models/` and `logs/` directories automatically
- Works in both Docker and local environments

## Running Locally

### Step 1: Stop Docker Service

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services
docker compose stop vision-service
```

### Step 2: Activate Virtual Environment

```bash
cd ~/Documents/ops/ai-project
source scripts/activate_venv.sh all
cd elderly_companion/ai_services
```

### Step 3: Run Service

**Option A: Direct execution**
```bash
python3 scripts/vision_service.py
```

**Option B: Using test script**
```bash
./test_vision_local.sh
```

**Option C: With custom port (if 8003 is in use)**
```bash
VISION_SERVICE_PORT=8004 python3 scripts/vision_service.py
```

### Step 4: Test the Service

In another terminal:

```bash
# Health check
curl http://localhost:8003/health | python3 -m json.tool

# Test detection (create a test image first)
python3 -c "from PIL import Image; Image.new('RGB', (640, 480), color='blue').save('test.jpg')"
curl -X POST http://localhost:8003/detect -F "file=@test.jpg" | python3 -m json.tool
```

## Expected Behavior

### ✅ If Local Execution Works:
- **Conclusion:** The segfault is Docker-specific
- **Next Steps:**
  1. Increase Docker memory/resources
  2. Try different Docker base images
  3. Use Docker with more resources allocated

### ❌ If Local Execution Also Crashes:
- **Conclusion:** The issue is with the library combination
- **Next Steps:**
  1. Try different PyTorch/ultralytics versions
  2. Consider alternative detection libraries
  3. Use cloud ML services

## Troubleshooting

### Port Already in Use
```bash
# Check what's using the port
lsof -i :8003

# Kill the process or use a different port
VISION_SERVICE_PORT=8004 python3 scripts/vision_service.py
```

### Model Download Issues
- Models are cached in `~/.ultralytics/` by default
- First run will download the model (~6MB)
- Subsequent runs use cached version

### Path Issues
- Service uses current working directory by default
- Set `APP_BASE_PATH` to use a different base directory:
  ```bash
  APP_BASE_PATH=/path/to/base python3 scripts/vision_service.py
  ```

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `APP_BASE_PATH` | Current directory | Base path for models, logs, config |
| `VISION_SERVICE_PORT` | 8003 | Port to run the service on |
| `YOLO_MODEL` | yolov8n.pt | YOLO model to use |
| `CUDA_VISIBLE_DEVICES` | "" | Forces CPU-only operation |

## Directory Structure

When running locally, the service creates:
```
ai_services/
├── models/          # Model files (created automatically)
├── logs/            # Log files (created automatically)
├── config/          # Configuration files (optional)
└── scripts/
    └── vision_service.py
```

## Comparing Docker vs Local

| Aspect | Docker | Local |
|--------|--------|-------|
| Paths | `/app/models` | `./models` |
| Port | 8003 (configurable) | 8003 (configurable) |
| Isolation | Full container isolation | Uses system Python |
| Debugging | View logs with `docker compose logs` | Direct console output |
| Performance | May have overhead | Direct system access |

## Next Steps After Testing

1. **Document results** - Does local execution work?
2. **If local works** - Focus on Docker-specific fixes
3. **If local crashes** - Focus on library compatibility
4. **Share findings** - Help improve the configuration

