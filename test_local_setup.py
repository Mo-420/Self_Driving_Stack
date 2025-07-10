#!/usr/bin/env python3
"""
Local test script for WaveShare Robotaxi
Tests basic dependencies and functionality that can run on macOS
"""

import sys
import os

def test_basic_imports():
    """Test basic Python imports that should work on macOS"""
    print("Testing basic imports...")
    
    try:
        import yaml
        print("✓ PyYAML imported successfully")
    except ImportError as e:
        print(f"✗ PyYAML import failed: {e}")
    
    try:
        import flask
        print("✓ Flask imported successfully")
    except ImportError as e:
        print(f"✗ Flask import failed: {e}")
    
    try:
        import cv2
        print("✓ OpenCV imported successfully")
    except ImportError as e:
        print(f"✗ OpenCV import failed: {e}")
    
    try:
        import numpy as np
        print("✓ NumPy imported successfully")
    except ImportError as e:
        print(f"✗ NumPy import failed: {e}")

def test_config_parsing():
    """Test if we can parse the config file"""
    print("\nTesting config file parsing...")
    
    config_path = "ugv_rpi/config.yaml"
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                import yaml
                config = yaml.safe_load(f)
                print("✓ Config file parsed successfully")
                print(f"  Robot name: {config.get('base_config', {}).get('robot_name', 'Unknown')}")
                print(f"  SBC version: {config.get('base_config', {}).get('sbc_version', 'Unknown')}")
        except Exception as e:
            print(f"✗ Config parsing failed: {e}")
    else:
        print("✗ Config file not found")

def test_app_structure():
    """Test if the main app file can be parsed"""
    print("\nTesting app structure...")
    
    app_path = "ugv_rpi/app.py"
    if os.path.exists(app_path):
        try:
            with open(app_path, 'r') as f:
                content = f.read()
                if "Flask" in content and "BaseController" in content:
                    print("✓ App file structure looks correct")
                else:
                    print("✗ App file missing expected components")
        except Exception as e:
            print(f"✗ App file reading failed: {e}")
    else:
        print("✗ App file not found")

def generate_pi_setup_instructions():
    """Generate instructions for Pi setup"""
    print("\n" + "="*60)
    print("RASPBERRY PI SETUP INSTRUCTIONS")
    print("="*60)
    print("""
1. Transfer the ugv_rpi folder to your Raspberry Pi
2. SSH into your Pi: ssh pi@<your_pi_ip>
3. Navigate to the ugv_rpi directory: cd ugv_rpi
4. Make setup script executable: sudo chmod +x setup.sh
5. Run the setup: sudo ./setup.sh
6. Make autorun script executable: sudo chmod +x autorun.sh
7. Configure auto-start: ./autorun.sh
8. Reboot the Pi: sudo reboot
9. Access the web interface: http://<your_pi_ip>:5000
10. Access JupyterLab: http://<your_pi_ip>:8888

TROUBLESHOOTING:
- If motors don't respond: Check UART connections (TX→RX, RX→TX)
- If camera doesn't work: Enable camera in raspi-config
- If web interface is inaccessible: Check if Flask service is running
""")

if __name__ == "__main__":
    print("WaveShare Robotaxi - Local Setup Test")
    print("="*40)
    
    test_basic_imports()
    test_config_parsing()
    test_app_structure()
    generate_pi_setup_instructions() 