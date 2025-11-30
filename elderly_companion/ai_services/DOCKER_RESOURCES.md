# Docker Resource Configuration Guide

## Current Configuration

Your Docker Desktop is configured with:
- **CPUs:** 10 (maximum) ✅ Excellent
- **Memory:** Check your current setting (recommended: 8GB+)

## Resource Limits in docker-compose.yml

We've added resource limits to prevent segfaults and ensure stable operation:

### Vision Service (Most Resource-Intensive)
```yaml
deploy:
  resources:
    limits:
      memory: 4G      # Maximum memory
      cpus: '4'       # Maximum CPUs
    reservations:
      memory: 2G      # Guaranteed memory
      cpus: '2'       # Guaranteed CPUs
```

**Why:** YOLO/PyTorch needs significant memory for model inference. 4GB limit prevents memory-related segfaults.

### Speech Service
```yaml
deploy:
  resources:
    limits:
      memory: 2G
      cpus: '2'
    reservations:
      memory: 1G
      cpus: '1'
```

**Why:** Whisper models can be memory-intensive, especially larger models.

### LLM Service
```yaml
deploy:
  resources:
    limits:
      memory: 1G
      cpus: '1'
    reservations:
      memory: 512M
      cpus: '0.5'
```

**Why:** LLM service is lightweight (rule-based), needs minimal resources.

## Total Resource Requirements

**Minimum Recommended:**
- Memory: 7GB (2G + 1G + 4G)
- CPUs: 7 (2 + 1 + 4)

**With your 10 CPUs:** ✅ You have plenty of CPU resources!

## Docker Desktop Settings

### Recommended Settings

1. **Open Docker Desktop**
2. **Go to Settings → Resources → Advanced**
3. **Configure:**
   - **Memory:** 8GB minimum (12GB+ recommended if available)
   - **CPUs:** 10 (you already have this) ✅
   - **Swap:** 2GB
   - **Disk image size:** 64GB+ (for models and images)

4. **Click "Apply & Restart"**

### Why These Settings?

- **Memory:** Vision service needs 4GB, plus overhead for other services
- **CPUs:** You have 10, which is excellent for parallel processing
- **Swap:** Provides buffer if memory is temporarily exceeded
- **Disk:** Models (YOLO ~6MB, Whisper models can be 100MB+) need space

## Additional Optimizations Applied

### Environment Variables for Vision Service

```yaml
environment:
  - CUDA_VISIBLE_DEVICES=""      # Force CPU-only
  - TORCH_DEVICE=cpu              # Explicit CPU device
  - OMP_NUM_THREADS=2             # Limit OpenMP threads
  - MKL_NUM_THREADS=2             # Limit MKL threads
  - NUMEXPR_NUM_THREADS=2         # Limit NumExpr threads
```

**Why:** Limits threading to prevent conflicts that can cause segfaults.

### Health Check Extended Start Period

```yaml
healthcheck:
  start_period: 60s  # Give vision service more time to start
```

**Why:** Vision service needs time to download/load the YOLO model on first start.

## Testing the Configuration

### Step 1: Rebuild with New Settings

```bash
cd ~/Documents/ops/ai-project/elderly_companion/ai_services
docker compose down
docker compose build
docker compose up -d
```

### Step 2: Monitor Resource Usage

```bash
# Check container resource usage
docker stats

# Check specific service
docker stats companion-vision-service
```

### Step 3: Test Vision Service

```bash
# Health check
curl http://localhost:8003/health

# Test detection
python3 -c "from PIL import Image; Image.new('RGB', (640, 480), color='blue').save('test.jpg')"
curl -X POST http://localhost:8003/detect -F "file=@test.jpg" | python3 -m json.tool
```

## Troubleshooting

### If Vision Service Still Crashes

1. **Check Docker Memory:**
   ```bash
   docker stats companion-vision-service
   ```
   - If memory usage hits the limit, increase it in docker-compose.yml

2. **Check Logs:**
   ```bash
   docker compose logs vision-service
   ```

3. **Try Increasing Memory Limit:**
   ```yaml
   deploy:
     resources:
       limits:
         memory: 6G  # Increase from 4G
   ```

4. **Use Local Execution:**
   - Since local execution works, use it for development
   - Only use Docker for production deployment

### If Services Won't Start

1. **Check Docker Desktop is Running:**
   ```bash
   docker ps
   ```

2. **Check Available Resources:**
   - Docker Desktop → Settings → Resources
   - Ensure you have enough memory allocated

3. **Reduce Resource Limits Temporarily:**
   - Lower limits in docker-compose.yml
   - Test if services start
   - Gradually increase to find optimal settings

## Resource Monitoring

### View Real-Time Usage

```bash
# All containers
docker stats

# Specific service
docker stats companion-vision-service --no-stream

# Continuous monitoring
watch -n 1 'docker stats --no-stream'
```

### Check Container Limits

```bash
# Check vision service limits
docker inspect companion-vision-service | grep -A 10 "Resources"
```

## Best Practices

1. **Monitor First:** Watch resource usage during normal operation
2. **Adjust Gradually:** Don't set limits too high initially
3. **Leave Headroom:** Don't allocate 100% of available resources
4. **Test Changes:** Always test after changing resource limits
5. **Document Settings:** Keep track of what works for your system

## Current Status

✅ **CPU Resources:** Excellent (10 CPUs available)
⚠️ **Memory:** Check your Docker Desktop memory setting
✅ **Resource Limits:** Configured in docker-compose.yml
✅ **Threading Limits:** Configured to prevent conflicts
✅ **Local Execution:** Works perfectly (use for development)

## Next Steps

1. **Verify Docker Memory:** Ensure 8GB+ allocated in Docker Desktop
2. **Rebuild Services:** Apply new resource limits
3. **Test Vision Service:** See if segfault is resolved
4. **Monitor Usage:** Check resource consumption during operation

