#!/bin/bash
# Test vision service locally

cd "$(dirname "$0")"

echo "Testing vision service locally..."
echo "Make sure Docker vision service is stopped: docker compose stop vision-service"
echo ""

# Check if port is in use
if lsof -Pi :8003 -sTCP:LISTEN -t >/dev/null ; then
    echo "⚠️  Port 8003 is still in use. Stopping Docker service..."
    docker compose stop vision-service 2>/dev/null || true
    sleep 2
fi

# Start service in background
echo "Starting vision service..."
python3 scripts/vision_service.py &
SERVICE_PID=$!

# Wait for service to start
sleep 5

# Check if service is running
if ps -p $SERVICE_PID > /dev/null; then
    echo "✅ Service started successfully (PID: $SERVICE_PID)"
    echo ""
    echo "Testing health endpoint..."
    curl -s http://localhost:8003/health | python3 -m json.tool || echo "❌ Health check failed"
    echo ""
    echo "Service is running. Press Ctrl+C to stop, or run: kill $SERVICE_PID"
    wait $SERVICE_PID
else
    echo "❌ Service failed to start. Check logs above."
    exit 1
fi

