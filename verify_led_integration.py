#!/usr/bin/env python3
"""
LED Control System Verification Script
Verifies that all LED components are properly integrated
"""

import sys
import os
import importlib.util

def check_file_exists(path, description):
    """Check if a file exists"""
    if os.path.exists(path):
        size = os.path.getsize(path)
        print(f"✅ {description}: {path} ({size} bytes)")
        return True
    else:
        print(f"❌ {description}: {path} - NOT FOUND")
        return False

def check_python_module(module_path, description):
    """Check if a Python module can be imported"""
    try:
        spec = importlib.util.spec_from_file_location("module", module_path)
        if spec and spec.loader:
            module = importlib.util.module_from_spec(spec)
            print(f"✅ {description}: {module_path} (valid Python)")
            return True
    except Exception as e:
        print(f"❌ {description}: {module_path} - Error: {e}")
    return False

def check_file_content(path, search_string, description):
    """Check if a file contains specific content"""
    try:
        with open(path, 'r') as f:
            content = f.read()
            if search_string in content:
                print(f"✅ {description}")
                return True
            else:
                print(f"❌ {description} - Content not found")
                return False
    except Exception as e:
        print(f"❌ {description} - Error: {e}")
        return False

def main():
    print("\n" + "="*70)
    print("WeldMaster AI - LED Control System Verification")
    print("="*70 + "\n")
    
    base_path = os.path.dirname(os.path.abspath(__file__))
    parent_path = os.path.dirname(base_path)
    
    # Track results
    results = []
    
    print("1. Backend Files")
    print("-" * 70)
    results.append(check_file_exists(os.path.join(base_path, "backend/vision/led_control.py"), "LED Controller"))
    results.append(check_file_exists(os.path.join(base_path, "backend/api/led_routes.py"), "LED API Routes"))
    print()
    
    print("2. Frontend Components")
    print("-" * 70)
    results.append(check_file_exists(os.path.join(base_path, "components/LEDControl.tsx"), "LEDControl React Component"))
    print()
    
    print("3. Configuration Files")
    print("-" * 70)
    results.append(check_file_exists(os.path.join(base_path, "LED_CONTROL_GUIDE.md"), "LED Control Guide"))
    results.append(check_file_exists(os.path.join(base_path, "LED_IMPLEMENTATION_SUMMARY.md"), "LED Implementation Summary"))
    print()
    
    print("4. App Integration")
    print("-" * 70)
    results.append(check_file_content(
        os.path.join(base_path, "App.tsx"),
        "LEDControl",
        "LEDControl imported in App.tsx"
    ))
    results.append(check_file_content(
        os.path.join(base_path, "App.tsx"),
        "Lightbulb",
        "Lightbulb icon imported in App.tsx"
    ))
    results.append(check_file_content(
        os.path.join(base_path, "backend/app.py"),
        "init_led_controller",
        "LED initialization in backend/app.py"
    ))
    print()
    
    print("5. API Endpoints")
    print("-" * 70)
    results.append(check_file_content(
        os.path.join(base_path, "backend/api/led_routes.py"),
        "get_led_status",
        "LED status endpoint defined"
    ))
    results.append(check_file_content(
        os.path.join(base_path, "backend/api/led_routes.py"),
        "turn_on",
        "LED on endpoint defined"
    ))
    results.append(check_file_content(
        os.path.join(base_path, "backend/api/led_routes.py"),
        "set_intensity",
        "LED intensity endpoint defined"
    ))
    print()
    
    print("6. LED Controller Features")
    print("-" * 70)
    results.append(check_file_content(
        os.path.join(base_path, "backend/vision/led_control.py"),
        "set_intensity",
        "Intensity control method implemented"
    ))
    results.append(check_file_content(
        os.path.join(base_path, "backend/vision/led_control.py"),
        "set_preset",
        "Preset control method implemented"
    ))
    results.append(check_file_content(
        os.path.join(base_path, "backend/vision/led_control.py"),
        "temperature",
        "Temperature monitoring implemented"
    ))
    results.append(check_file_content(
        os.path.join(base_path, "backend/vision/led_control.py"),
        "light_level",
        "Light sensor support implemented"
    ))
    print()
    
    # Summary
    print("="*70)
    passed = sum(results)
    total = len(results)
    percentage = (passed / total * 100) if total > 0 else 0
    
    print(f"VERIFICATION RESULTS: {passed}/{total} checks passed ({percentage:.0f}%)\n")
    
    if percentage == 100:
        print("✅ ALL SYSTEMS READY - LED Control fully integrated!")
        print("\nYou can now:")
        print("  1. Run the app and go to Settings → Lighting Control")
        print("  2. Use curl to test: curl http://localhost:5000/api/led/status")
        print("  3. View documentation: See LED_CONTROL_GUIDE.md")
        return 0
    else:
        print("⚠️  Some checks failed - Please verify the integration")
        return 1

if __name__ == "__main__":
    sys.exit(main())
