#!/usr/bin/env python3
"""
Simple test client for AI services
Tests all three services: Speech, LLM, and Vision
"""
import requests
import json
import sys

BASE_URL = "http://localhost"

def print_section(title):
    """Print a formatted section header"""
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}\n")

def test_speech_service():
    """Test speech service"""
    print_section("Testing Speech Service")
    try:
        response = requests.get(f"{BASE_URL}:8001/health", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"✅ Service Status: {data.get('status', 'unknown')}")
            print(f"   Model Loaded: {data.get('model_loaded', False)}")
            print(f"   ROS 2 Available: {data.get('ros2_available', False)}")
        else:
            print(f"❌ Service returned status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"❌ Error connecting to speech service: {e}")
        return False
    return True

def test_llm_service():
    """Test LLM service"""
    print_section("Testing LLM Service")
    try:
        # Health check
        response = requests.get(f"{BASE_URL}:8002/health", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"✅ Service Status: {data.get('status', 'unknown')}")
            print(f"   Personality Loaded: {data.get('personality_loaded', False)}")
            print(f"   ROS 2 Available: {data.get('ros2_available', False)}")
        else:
            print(f"❌ Service returned status code: {response.status_code}")
            return False
        
        # Test chat
        print("\n💬 Testing Chat Functionality:")
        test_messages = [
            "Hello!",
            "I feel a bit lonely today",
            "Can you remind me about my medicine?",
            "How are you doing?"
        ]
        
        for i, message in enumerate(test_messages, 1):
            try:
                chat_response = requests.post(
                    f"{BASE_URL}:8002/chat",
                    json={"message": message, "user_id": "test_user"},
                    timeout=10
                )
                if chat_response.status_code == 200:
                    data = chat_response.json()
                    print(f"\n   Test {i}:")
                    print(f"   👤 User: {message}")
                    print(f"   🤖 Bot: {data.get('response', 'No response')}")
                else:
                    print(f"   ❌ Chat request {i} failed: {chat_response.status_code}")
            except requests.exceptions.RequestException as e:
                print(f"   ❌ Error in chat request {i}: {e}")
        
    except requests.exceptions.RequestException as e:
        print(f"❌ Error connecting to LLM service: {e}")
        return False
    return True

def test_vision_service():
    """Test vision service"""
    print_section("Testing Vision Service")
    try:
        response = requests.get(f"{BASE_URL}:8003/health", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"✅ Service Status: {data.get('status', 'unknown')}")
            print(f"   Model Loaded: {data.get('model_loaded', False)}")
            print(f"   ROS 2 Available: {data.get('ros2_available', False)}")
            print("\n💡 To test object detection, use:")
            print("   curl -X POST http://localhost:8003/detect -F 'file=@/path/to/image.jpg'")
        else:
            print(f"❌ Service returned status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"❌ Error connecting to vision service: {e}")
        return False
    return True

def test_api_docs():
    """Check if API documentation is accessible"""
    print_section("API Documentation")
    services = [
        ("Speech", 8001),
        ("LLM", 8002),
        ("Vision", 8003)
    ]
    
    for name, port in services:
        try:
            response = requests.get(f"{BASE_URL}:{port}/docs", timeout=5)
            if response.status_code == 200:
                print(f"✅ {name} Service API Docs: http://localhost:{port}/docs")
            else:
                print(f"❌ {name} Service API Docs: Not accessible")
        except requests.exceptions.RequestException:
            print(f"❌ {name} Service API Docs: Not accessible")

def main():
    """Run all tests"""
    print("\n" + "="*60)
    print("  AI Services Test Client")
    print("="*60)
    
    results = {
        "Speech": test_speech_service(),
        "LLM": test_llm_service(),
        "Vision": test_vision_service()
    }
    
    test_api_docs()
    
    # Summary
    print_section("Test Summary")
    all_passed = all(results.values())
    for service, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"  {service} Service: {status}")
    
    if all_passed:
        print("\n🎉 All services are working correctly!")
        return 0
    else:
        print("\n⚠️  Some services have issues. Check the logs above.")
        return 1

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        sys.exit(1)

