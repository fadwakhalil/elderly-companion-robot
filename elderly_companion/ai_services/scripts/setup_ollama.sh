#!/bin/bash
# Setup script for Ollama integration

set -e

echo "=========================================="
echo "Setting up Ollama for Local LLM"
echo "=========================================="
echo ""

# Check if Ollama is installed
if ! command -v ollama &> /dev/null; then
    echo "Installing Ollama..."
    brew install ollama
else
    echo "✅ Ollama is already installed"
fi

# Start Ollama service
echo ""
echo "Starting Ollama service..."
echo "Note: This will run in the background. To stop: pkill ollama"
ollama serve > /dev/null 2>&1 &
OLLAMA_PID=$!

# Wait for Ollama to start
echo "Waiting for Ollama to start..."
sleep 5

# Check if Ollama is running
if curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
    echo "✅ Ollama is running"
else
    echo "❌ Ollama failed to start"
    exit 1
fi

# Pull recommended models
echo ""
echo "Available models:"
echo "  1. llama3.2:1b  - Smallest, fastest (good for Jetson)"
echo "  2. llama3.2:3b  - Balanced (good for testing)"
echo "  3. llama3.2    - Full model (best quality, needs more resources)"
echo ""
read -p "Which model to download? (1/2/3) [2]: " model_choice
model_choice=${model_choice:-2}

case $model_choice in
    1)
        MODEL="llama3.2:1b"
        ;;
    2)
        MODEL="llama3.2:3b"
        ;;
    3)
        MODEL="llama3.2"
        ;;
    *)
        MODEL="llama3.2:3b"
        ;;
esac

echo ""
echo "Downloading $MODEL..."
echo "This may take a few minutes..."
ollama pull $MODEL

echo ""
echo "✅ Setup complete!"
echo ""
echo "Test the model:"
echo "  ollama run $MODEL 'Hello, how are you?'"
echo ""
echo "Ollama is running in the background (PID: $OLLAMA_PID)"
echo "To stop: pkill ollama"

