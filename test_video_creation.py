#!/usr/bin/env python3
"""
Simple test script to verify video creation functionality.
"""
import requests
import json
import time

def test_video_creation():
    """Test the post-simulation video creation system."""
    
    print("Testing Post-Simulation Video Creation System")
    print("=" * 50)
    
    # 1. Test template endpoint
    print("1. Testing template endpoint...")
    try:
        response = requests.get("http://localhost:5000/api/video/templates")
        if response.status_code == 200:
            templates = response.json()
            print(f"✓ Templates available: {list(templates['templates'].keys())}")
        else:
            print(f"✗ Template endpoint failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Template endpoint error: {e}")
        return False
    
    # 2. Start a simulation
    print("\n2. Starting simulation...")
    try:
        start_payload = {
            "drone": "Quadcopter X4",
            "environment": "Earth",
            "mission": "reconnaissance"
        }
        response = requests.post(
            "http://localhost:5000/api/simulation/start",
            headers={"Content-Type": "application/json"},
            data=json.dumps(start_payload)
        )
        if response.status_code == 200:
            result = response.json()
            print(f"✓ Simulation started: {result.get('message')}")
        else:
            print(f"✗ Simulation start failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Simulation start error: {e}")
        return False
    
    # 3. Wait for telemetry
    print("\n3. Waiting for telemetry data...")
    time.sleep(10)
    
    # 4. Stop simulation
    print("\n4. Stopping simulation...")
    try:
        response = requests.post("http://localhost:5000/api/simulation/stop")
        if response.status_code == 200:
            print("✓ Simulation stopped")
        else:
            print(f"✗ Simulation stop failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Simulation stop error: {e}")
        return False
    
    # 5. Check telemetry data
    print("\n5. Checking telemetry data...")
    try:
        response = requests.get("http://localhost:5000/api/simulation/telemetry")
        if response.status_code == 200:
            telemetry = response.json()
            print(f"✓ Telemetry data available: {len(telemetry)} points")
            if len(telemetry) > 0:
                print(f"  Sample point: {telemetry[0]}")
        else:
            print(f"✗ Telemetry check failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Telemetry check error: {e}")
        return False
    
    # 6. Test video creation
    print("\n6. Testing video creation...")
    try:
        video_payload = {
            "template": "professional",
            "include_audio": False  # Skip audio for faster testing
        }
        response = requests.post(
            "http://localhost:5000/api/video/create/current",
            headers={"Content-Type": "application/json"},
            data=json.dumps(video_payload),
            timeout=30
        )
        print(f"Response status: {response.status_code}")
        print(f"Response content: {response.text}")
        
        if response.status_code == 200:
            result = response.json()
            if result.get('success'):
                print(f"✓ Video created successfully: {result.get('filename')}")
                print(f"  Duration: {result.get('duration_seconds')}s")
                print(f"  Size: {result.get('file_size_mb')}MB")
                return True
            else:
                print(f"✗ Video creation failed: {result.get('error')}")
                return False
        else:
            print(f"✗ Video creation request failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Video creation error: {e}")
        return False

if __name__ == "__main__":
    success = test_video_creation()
    if success:
        print("\n🎉 All tests passed! Video creation system is working.")
    else:
        print("\n❌ Some tests failed. Check the implementation.")