#!/bin/bash
# Test script for Docker configuration with resource limits

set -e

echo "=========================================="
echo "Testing Docker Configuration"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Step 1: Check Docker is running
echo -e "${YELLOW}Step 1: Checking Docker...${NC}"
if ! docker ps > /dev/null 2>&1; then
    echo -e "${RED}❌ Docker is not running. Please start Docker Desktop.${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Docker is running${NC}"
echo ""

# Step 2: Stop existing services
echo -e "${YELLOW}Step 2: Stopping existing services...${NC}"
docker compose down
echo -e "${GREEN}✅ Services stopped${NC}"
echo ""

# Step 3: Check Docker resources
echo -e "${YELLOW}Step 3: Checking Docker resource allocation...${NC}"
echo "Please verify in Docker Desktop:"
echo "  - Memory: 8GB+ recommended"
echo "  - CPUs: 10 (you have this ✅)"
echo ""
read -p "Press Enter to continue after verifying Docker resources..."
echo ""

# Step 4: Build vision service
echo -e "${YELLOW}Step 4: Building vision service...${NC}"
docker compose build vision-service
echo -e "${GREEN}✅ Vision service built${NC}"
echo ""

# Step 5: Start services
echo -e "${YELLOW}Step 5: Starting services...${NC}"
docker compose up -d
echo -e "${GREEN}✅ Services started${NC}"
echo ""

# Step 6: Wait for services to be ready
echo -e "${YELLOW}Step 6: Waiting for services to be ready...${NC}"
echo "Waiting 15 seconds for services to initialize..."
sleep 15

# Check service status
echo ""
echo -e "${YELLOW}Service Status:${NC}"
docker compose ps
echo ""

# Step 7: Test health endpoints
echo -e "${YELLOW}Step 7: Testing health endpoints...${NC}"

test_health() {
    local service=$1
    local port=$2
    local name=$3
    
    if curl -s -f http://localhost:${port}/health > /dev/null 2>&1; then
        echo -e "${GREEN}✅ ${name} service is healthy${NC}"
        curl -s http://localhost:${port}/health | python3 -m json.tool 2>/dev/null || echo "  (Response received)"
        return 0
    else
        echo -e "${RED}❌ ${name} service is not responding${NC}"
        return 1
    fi
}

test_health "speech" "8001" "Speech"
test_health "llm" "8002" "LLM"
test_health "vision" "8003" "Vision"

echo ""

# Step 8: Monitor resource usage
echo -e "${YELLOW}Step 8: Checking resource usage...${NC}"
echo "Resource usage (press Ctrl+C to stop monitoring):"
echo ""
docker stats --no-stream companion-vision-service companion-speech-service companion-llm-service
echo ""

# Step 9: Test vision detection
echo -e "${YELLOW}Step 9: Testing vision detection...${NC}"

# Create test image
if [ ! -f "test_detection.jpg" ]; then
    python3 -c "from PIL import Image; Image.new('RGB', (640, 480), color='blue').save('test_detection.jpg')"
    echo "Created test image: test_detection.jpg"
fi

echo "Sending detection request..."
response=$(curl -s -w "\n%{http_code}" -X POST http://localhost:8003/detect -F "file=@test_detection.jpg" 2>&1)
http_code=$(echo "$response" | tail -n1)
body=$(echo "$response" | sed '$d')

if [ "$http_code" = "200" ]; then
    echo -e "${GREEN}✅ Detection successful!${NC}"
    echo "$body" | python3 -m json.tool 2>/dev/null || echo "$body"
else
    echo -e "${RED}❌ Detection failed (HTTP $http_code)${NC}"
    echo "Response: $body"
    echo ""
    echo "Checking logs..."
    docker compose logs vision-service --tail=20
fi

echo ""

# Step 10: Check logs for errors
echo -e "${YELLOW}Step 10: Checking for errors in logs...${NC}"
echo "Recent vision service logs:"
docker compose logs vision-service --tail=10 | grep -i "error\|exception\|segfault\|crash" || echo "No errors found in recent logs"

echo ""
echo "=========================================="
echo -e "${GREEN}Testing Complete!${NC}"
echo "=========================================="
echo ""
echo "To monitor resources continuously:"
echo "  docker stats"
echo ""
echo "To view logs:"
echo "  docker compose logs -f vision-service"
echo ""
echo "To stop services:"
echo "  docker compose down"

