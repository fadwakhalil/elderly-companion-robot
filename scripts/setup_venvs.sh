#!/bin/bash
# Setup script for creating and managing virtual environments
# Usage: ./setup_venvs.sh [create|activate|deactivate|list]

set -e

COMPANION_DIR="$HOME/Documents/ops/ai-project/elderly_companion"
AI_SERVICES_DIR="$COMPANION_DIR/ai_services"
ROS2_WS_DIR="$COMPANION_DIR/ros2_workspace"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

create_venvs() {
    echo -e "${BLUE}Creating separate virtual environments for each service...${NC}"
    
    # Create AI service virtual environments
    cd "$AI_SERVICES_DIR"
    
    echo -e "${GREEN}Creating speech service venv...${NC}"
    python3 -m venv venv_speech
    
    echo -e "${GREEN}Creating LLM service venv...${NC}"
    python3 -m venv venv_llm
    
    echo -e "${GREEN}Creating vision service venv...${NC}"
    python3 -m venv venv_vision
    
    # Create ROS 2 development venv
    cd "$ROS2_WS_DIR"
    echo -e "${GREEN}Creating ROS 2 development venv...${NC}"
    python3 -m venv venv_ros2_dev
    
    echo -e "${GREEN}All virtual environments created!${NC}"
    echo -e "${YELLOW}Next step: Install dependencies with 'install_deps' command${NC}"
}

create_single_venv() {
    echo -e "${BLUE}Creating single virtual environment for all services...${NC}"
    
    cd "$AI_SERVICES_DIR"
    
    echo -e "${GREEN}Creating unified virtual environment...${NC}"
    python3 -m venv venv
    
    echo -e "${GREEN}Virtual environment created!${NC}"
    echo -e "${YELLOW}Next step: Install dependencies with 'install_deps_single' command${NC}"
}

install_deps() {
    echo -e "${BLUE}Installing dependencies in virtual environments...${NC}"
    
    cd "$AI_SERVICES_DIR"
    
    # Check if requirements files exist
    if [ ! -f "requirements_speech.txt" ]; then
        echo -e "${YELLOW}Warning: requirements_speech.txt not found. Skipping speech service.${NC}"
    else
        echo -e "${GREEN}Installing speech service dependencies...${NC}"
        source venv_speech/bin/activate
        pip install --upgrade pip
        pip install -r requirements_speech.txt
        deactivate
    fi
    
    if [ ! -f "requirements_llm.txt" ]; then
        echo -e "${YELLOW}Warning: requirements_llm.txt not found. Skipping LLM service.${NC}"
    else
        echo -e "${GREEN}Installing LLM service dependencies...${NC}"
        source venv_llm/bin/activate
        pip install --upgrade pip
        pip install -r requirements_llm.txt
        deactivate
    fi
    
    if [ ! -f "requirements_vision.txt" ]; then
        echo -e "${YELLOW}Warning: requirements_vision.txt not found. Skipping vision service.${NC}"
    else
        echo -e "${GREEN}Installing vision service dependencies...${NC}"
        source venv_vision/bin/activate
        pip install --upgrade pip
        pip install -r requirements_vision.txt
        deactivate
    fi
    
    echo -e "${GREEN}Dependencies installed!${NC}"
}

install_deps_single() {
    echo -e "${BLUE}Installing all dependencies in single virtual environment...${NC}"
    
    cd "$AI_SERVICES_DIR"
    
    if [ ! -d "venv" ]; then
        echo -e "${YELLOW}Error: venv not found. Create it first with 'create_single' command${NC}"
        return 1
    fi
    
    source venv/bin/activate
    pip install --upgrade pip
    
    # Install all requirements
    if [ -f "requirements_speech.txt" ]; then
        echo -e "${GREEN}Installing speech service dependencies...${NC}"
        pip install -r requirements_speech.txt
    fi
    
    if [ -f "requirements_llm.txt" ]; then
        echo -e "${GREEN}Installing LLM service dependencies...${NC}"
        pip install -r requirements_llm.txt
    fi
    
    if [ -f "requirements_vision.txt" ]; then
        echo -e "${GREEN}Installing vision service dependencies...${NC}"
        pip install -r requirements_vision.txt
    fi
    
    deactivate
    
    echo -e "${GREEN}All dependencies installed in single virtual environment!${NC}"
}

activate_venv() {
    SERVICE=$1
    
    case $SERVICE in
        all|single)
            if [ -d "$AI_SERVICES_DIR/venv" ]; then
                source "$AI_SERVICES_DIR/venv/bin/activate"
                echo -e "${GREEN}Activated unified virtual environment (all services)${NC}"
            else
                echo -e "${YELLOW}Error: Single venv not found. Run: ./setup_venvs.sh create_single${NC}"
                return 1
            fi
            ;;
        speech)
            if [ -d "$AI_SERVICES_DIR/venv_speech" ]; then
                source "$AI_SERVICES_DIR/venv_speech/bin/activate"
                echo -e "${GREEN}Activated speech service virtual environment${NC}"
            else
                echo -e "${YELLOW}Error: venv_speech not found. Run: ./setup_venvs.sh create${NC}"
                return 1
            fi
            ;;
        llm)
            if [ -d "$AI_SERVICES_DIR/venv_llm" ]; then
                source "$AI_SERVICES_DIR/venv_llm/bin/activate"
                echo -e "${GREEN}Activated LLM service virtual environment${NC}"
            else
                echo -e "${YELLOW}Error: venv_llm not found. Run: ./setup_venvs.sh create${NC}"
                return 1
            fi
            ;;
        vision)
            if [ -d "$AI_SERVICES_DIR/venv_vision" ]; then
                source "$AI_SERVICES_DIR/venv_vision/bin/activate"
                echo -e "${GREEN}Activated vision service virtual environment${NC}"
            else
                echo -e "${YELLOW}Error: venv_vision not found. Run: ./setup_venvs.sh create${NC}"
                return 1
            fi
            ;;
        ros2)
            if [ -d "$ROS2_WS_DIR/venv_ros2_dev" ]; then
                source "$ROS2_WS_DIR/venv_ros2_dev/bin/activate"
                echo -e "${GREEN}Activated ROS 2 development virtual environment${NC}"
                echo -e "${YELLOW}Remember to source ROS 2:${NC}"
                echo -e "  source /opt/homebrew/opt/ros/humble/setup.zsh"
                echo -e "  source $ROS2_WS_DIR/install/setup.zsh"
            else
                echo -e "${YELLOW}Error: venv_ros2_dev not found. Run: ./setup_venvs.sh create${NC}"
                return 1
            fi
            ;;
        *)
            echo -e "${YELLOW}Usage: source $0 activate [all|speech|llm|vision|ros2]${NC}"
            echo -e "${YELLOW}Available environments:${NC}"
            list_venvs
            return 1
            ;;
    esac
}

list_venvs() {
    echo -e "${BLUE}Available virtual environments:${NC}"
    echo ""
    
    # Single unified venv
    if [ -d "$AI_SERVICES_DIR/venv" ]; then
        echo -e "${GREEN}✓${NC} all/single - $AI_SERVICES_DIR/venv (unified for all services)"
    else
        echo -e "${YELLOW}✗${NC} all/single - Not created (recommended: ./setup_venvs.sh create_single)"
    fi
    echo ""
    
    # Separate venvs
    if [ -d "$AI_SERVICES_DIR/venv_speech" ]; then
        echo -e "${GREEN}✓${NC} speech - $AI_SERVICES_DIR/venv_speech"
    else
        echo -e "${YELLOW}✗${NC} speech - Not created"
    fi
    
    if [ -d "$AI_SERVICES_DIR/venv_llm" ]; then
        echo -e "${GREEN}✓${NC} llm - $AI_SERVICES_DIR/venv_llm"
    else
        echo -e "${YELLOW}✗${NC} llm - Not created"
    fi
    
    if [ -d "$AI_SERVICES_DIR/venv_vision" ]; then
        echo -e "${GREEN}✓${NC} vision - $AI_SERVICES_DIR/venv_vision"
    else
        echo -e "${YELLOW}✗${NC} vision - Not created"
    fi
    
    if [ -d "$ROS2_WS_DIR/venv_ros2_dev" ]; then
        echo -e "${GREEN}✓${NC} ros2 - $ROS2_WS_DIR/venv_ros2_dev"
    else
        echo -e "${YELLOW}✗${NC} ros2 - Not created"
    fi
}

show_usage() {
    echo "Virtual Environment Management Script"
    echo ""
    echo "Usage:"
    echo "  ./setup_venvs.sh create          - Create separate venvs for each service"
    echo "  ./setup_venvs.sh create_single   - Create single venv for all services (recommended)"
    echo "  ./setup_venvs.sh install_deps    - Install dependencies in separate venvs"
    echo "  ./setup_venvs.sh install_deps_single - Install all deps in single venv"
    echo "  source ./setup_venvs.sh activate [service] - Activate a specific venv"
    echo "  ./setup_venvs.sh list            - List all virtual environments"
    echo ""
    echo "Services: speech, llm, vision, ros2, or 'all' for single venv"
    echo ""
    echo "Examples:"
    echo "  ./setup_venvs.sh create_single   # Recommended: single venv"
    echo "  ./setup_venvs.sh install_deps_single"
    echo "  source ./setup_venvs.sh activate all"
    echo ""
    echo "  ./setup_venvs.sh create          # Alternative: separate venvs"
    echo "  source ./setup_venvs.sh activate speech"
}

# Main command handler
case "${1:-}" in
    create)
        create_venvs
        ;;
    create_single)
        create_single_venv
        ;;
    install_deps)
        install_deps
        ;;
    install_deps_single)
        install_deps_single
        ;;
    activate)
        if [ -z "${2:-}" ]; then
            echo -e "${YELLOW}Error: Please specify a service${NC}"
            show_usage
            exit 1
        fi
        activate_venv "$2"
        ;;
    list)
        list_venvs
        ;;
    *)
        show_usage
        ;;
esac
