#!/usr/bin/env python3
"""
WeldMaster AI - System Startup Verification Script

This script verifies:
1. Database exists and is ready
2. RDK X5 hardware is detected
3. Camera/ROS2 connectivity
4. Backend is running and accessible
"""

import os
import sys
import time
import subprocess
from pathlib import Path

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from system_check import HardwareDetector, DatabaseManager, setup_logging
import logging

logger = setup_logging(logging.INFO)


def print_section(title):
    """Print formatted section header"""
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def check_database():
    """Verify database setup"""
    print_section("DATABASE CHECK")
    
    db_manager = DatabaseManager()
    try:
        db_manager.initialize()
        print("\n✅ Database Check: PASSED")
        for msg in db_manager.init_messages:
            print(f"   {msg}")
        return True
    except Exception as e:
        print(f"\n❌ Database Check: FAILED")
        print(f"   Error: {str(e)}")
        return False


def check_hardware():
    """Verify hardware detection"""
    print_section("HARDWARE CHECK")
    
    detector = HardwareDetector()
    status = detector.detect_all()
    
    print("\nDetection Results:")
    for msg in status['detection_messages']:
        print(f"   {msg}")
    
    print("\nSummary:")
    print(f"   RDK X5 Available: {'YES ✅' if status['rdk_x5_available'] else 'NO ⚠️'}")
    print(f"   ROS2 Available: {'YES ✅' if status['ros2_available'] else 'NO ⚠️'}")
    print(f"   Camera Connected: {'YES ✅' if status['camera_available'] else 'NO ⚠️'}")
    print(f"   Ready for Operation: {'YES ✅' if status['ready_for_operation'] else 'NO ⚠️'}")
    
    return status['ready_for_operation']


def check_dependencies():
    """Verify required Python packages"""
    print_section("DEPENDENCIES CHECK")
    
    required_packages = [
        ('flask', 'Flask'),
        ('flask_cors', 'Flask-CORS'),
        ('sqlalchemy', 'SQLAlchemy'),
        ('cv2', 'OpenCV'),
        ('numpy', 'NumPy'),
        ('yaml', 'PyYAML'),
    ]
    
    missing = []
    print("\nRequired Packages:")
    
    for module, name in required_packages:
        try:
            __import__(module)
            print(f"   ✅ {name}")
        except ImportError:
            print(f"   ❌ {name} - NOT INSTALLED")
            missing.append(name)
    
    if missing:
        print(f"\n❌ Missing packages: {', '.join(missing)}")
        print("   Run: pip install -r backend/requirements.txt")
        return False
    else:
        print("\n✅ All required packages installed")
        return True


def check_backend_startup():
    """Test if backend can start"""
    print_section("BACKEND STARTUP TEST")
    
    print("\nAttempting to import backend modules...")
    try:
        # Try to import backend modules
        from database.models import Student, Scan
        print("   ✅ Database models imported")
        
        from vision.evaluator import WeldEvaluator
        print("   ✅ Vision evaluator imported")
        
        from vision.calibration import Calibrator
        print("   ✅ Calibration module imported")
        
        print("\n✅ Backend modules: READY")
        return True
        
    except Exception as e:
        print(f"\n❌ Backend import failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def print_recommendations(db_ok, hardware_ok, deps_ok, backend_ok):
    """Print recommendations based on checks"""
    print_section("STARTUP RECOMMENDATIONS")
    
    all_ok = db_ok and hardware_ok and deps_ok and backend_ok
    
    if all_ok:
        print("\n✅ All checks passed! Ready to start backend.\n")
        print("   Start backend with:")
        print("   $ cd backend")
        print("   $ python app.py\n")
    else:
        print("\n⚠️ Some checks failed. Please address issues:\n")
        
        if not deps_ok:
            print("   1. Install dependencies:")
            print("      $ pip install -r backend/requirements.txt\n")
        
        if not db_ok:
            print("   2. Fix database issues:")
            print("      $ rm backend/weld_data.db  # Remove old database")
            print("      $ python backend/app.py    # Re-create\n")
        
        if not backend_ok:
            print("   3. Fix backend import issues:")
            print("      $ python -c 'from system_check import *'\n")
        
        if not hardware_ok:
            print("   4. Note: RDK X5/Camera not detected")
            print("      - This is OK for development")
            print("      - For RDK X5: Ensure ROS2 is installed\n")


def main():
    """Run all startup checks"""
    print("\n" + "=" * 70)
    print("  WeldMaster AI - System Startup Verification")
    print("  Horizon Robotics RDK X5 Welding Evaluation System")
    print("=" * 70)
    
    # Run checks
    db_ok = check_database()
    hardware_ok = check_hardware()
    deps_ok = check_dependencies()
    backend_ok = check_backend_startup()
    
    # Print recommendations
    print_recommendations(db_ok, hardware_ok, deps_ok, backend_ok)
    
    # Return exit code
    if db_ok and deps_ok and backend_ok:
        print("=" * 70)
        print("  System Ready for Startup")
        print("=" * 70 + "\n")
        return 0
    else:
        print("=" * 70)
        print("  System NOT Ready - Please fix issues above")
        print("=" * 70 + "\n")
        return 1


if __name__ == '__main__':
    try:
        exit_code = main()
        sys.exit(exit_code)
    except Exception as e:
        logger.error(f"Startup verification failed: {e}", exc_info=True)
        sys.exit(1)
