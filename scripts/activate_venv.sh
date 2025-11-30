#!/bin/bash
# Simple activation script for virtual environments
# Usage: source scripts/activate_venv.sh [speech|llm|vision|ros2]

COMPANION_DIR="$HOME/Documents/ops/ai-project/elderly_companion"
AI_SERVICES_DIR="$COMPANION_DIR/ai_services"
ROS2_WS_DIR="$COMPANION_DIR/ros2_workspace"

SERVICE=${1:-}

case $SERVICE in
  all|single)
    if [ -d "$AI_SERVICES_DIR/venv" ]; then
      source "$AI_SERVICES_DIR/venv/bin/activate"
      echo "✓ Activated unified virtual environment (all services)"
    else
      echo "✗ Error: Single venv not found. Run: ./scripts/setup_venvs.sh create_single"
      return 1
    fi
    ;;
  speech)
    if [ -d "$AI_SERVICES_DIR/venv_speech" ]; then
      source "$AI_SERVICES_DIR/venv_speech/bin/activate"
      echo "✓ Activated speech service virtual environment"
    else
      echo "✗ Error: venv_speech not found. Run: ./scripts/setup_venvs.sh create"
      return 1
    fi
    ;;
  llm)
    if [ -d "$AI_SERVICES_DIR/venv_llm" ]; then
      source "$AI_SERVICES_DIR/venv_llm/bin/activate"
      echo "✓ Activated LLM service virtual environment"
    else
      echo "✗ Error: venv_llm not found. Run: ./scripts/setup_venvs.sh create"
      return 1
    fi
    ;;
  vision)
    if [ -d "$AI_SERVICES_DIR/venv_vision" ]; then
      source "$AI_SERVICES_DIR/venv_vision/bin/activate"
      echo "✓ Activated vision service virtual environment"
    else
      echo "✗ Error: venv_vision not found. Run: ./scripts/setup_venvs.sh create"
      return 1
    fi
    ;;
  ros2)
    if [ -d "$ROS2_WS_DIR/venv_ros2_dev" ]; then
      source "$ROS2_WS_DIR/venv_ros2_dev/bin/activate"
      echo "✓ Activated ROS 2 development virtual environment"
      echo "  Remember to source ROS 2:"
      echo "    source /opt/homebrew/opt/ros/humble/setup.zsh"
      echo "    source $ROS2_WS_DIR/install/setup.zsh"
    else
      echo "✗ Error: venv_ros2_dev not found. Run: ./scripts/setup_venvs.sh create"
      return 1
    fi
    ;;
  *)
    echo "Usage: source scripts/activate_venv.sh [all|speech|llm|vision|ros2]"
    echo ""
    echo "Available environments:"
    [ -d "$AI_SERVICES_DIR/venv" ] && echo "  ✓ all/single (recommended)" || echo "  ✗ all/single (not created)"
    [ -d "$AI_SERVICES_DIR/venv_speech" ] && echo "  ✓ speech" || echo "  ✗ speech (not created)"
    [ -d "$AI_SERVICES_DIR/venv_llm" ] && echo "  ✓ llm" || echo "  ✗ llm (not created)"
    [ -d "$AI_SERVICES_DIR/venv_vision" ] && echo "  ✓ vision" || echo "  ✗ vision (not created)"
    [ -d "$ROS2_WS_DIR/venv_ros2_dev" ] && echo "  ✓ ros2" || echo "  ✗ ros2 (not created)"
    return 1
    ;;
esac
