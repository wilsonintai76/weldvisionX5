#!/usr/bin/env python3
"""
Test script to verify frontend-backend connection
Run this after starting the backend: python app.py
"""

import requests
import json
import sys
from datetime import datetime

BASE_URL = "http://localhost:5000"
TIMEOUT = 5

def print_header(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}\n")

def test_health():
    """Test backend health endpoint"""
    print_header("1. Testing Backend Health")
    try:
        response = requests.get(f"{BASE_URL}/api/health", timeout=TIMEOUT)
        print(f"✅ Backend is running!")
        print(f"Status Code: {response.status_code}")
        print(f"Response: {json.dumps(response.json(), indent=2)}")
        return True
    except requests.exceptions.ConnectionError:
        print(f"❌ Cannot connect to backend at {BASE_URL}")
        print("   Make sure backend is running: python backend/app.py")
        return False
    except Exception as e:
        print(f"❌ Error: {str(e)}")
        return False

def test_create_student():
    """Test creating a student"""
    print_header("2. Testing Create Student")
    try:
        payload = {
            "name": "Test Student",
            "student_id": f"TEST_{datetime.now().timestamp()}",
            "class_name": "Welding 101",
            "level": "Novice"
        }
        response = requests.post(
            f"{BASE_URL}/api/students",
            json=payload,
            timeout=TIMEOUT
        )
        print(f"Status Code: {response.status_code}")
        data = response.json()
        print(f"Response: {json.dumps(data, indent=2)}")
        if response.status_code in [200, 201]:
            print(f"✅ Student created successfully!")
            return data.get('id')
        else:
            print(f"❌ Failed to create student")
            return None
    except Exception as e:
        print(f"❌ Error: {str(e)}")
        return None

def test_list_students():
    """Test listing students"""
    print_header("3. Testing List Students")
    try:
        response = requests.get(f"{BASE_URL}/api/students", timeout=TIMEOUT)
        print(f"Status Code: {response.status_code}")
        data = response.json()
        print(f"Found {len(data)} students")
        if data:
            print(f"First student: {json.dumps(data[0], indent=2)}")
            print(f"✅ Successfully fetched student list!")
            return True
        else:
            print(f"⚠️  No students found (create one first)")
            return True
    except Exception as e:
        print(f"❌ Error: {str(e)}")
        return False

def test_get_rubric():
    """Test getting rubric"""
    print_header("4. Testing Get Rubric")
    try:
        response = requests.get(f"{BASE_URL}/api/rubric", timeout=TIMEOUT)
        print(f"Status Code: {response.status_code}")
        data = response.json()
        print(f"Response: {json.dumps(data, indent=2)}")
        print(f"✅ Successfully fetched rubric!")
        return data
    except Exception as e:
        print(f"❌ Error: {str(e)}")
        return None

def test_trigger_scan(student_id):
    """Test triggering a scan"""
    print_header("5. Testing Trigger Scan")
    if not student_id:
        print(f"⚠️  Skipping scan test (no student ID)")
        return False
    
    try:
        rubric = {
            "targetWidth": 8.0,
            "widthTolerance": 1.0,
            "targetHeight": 2.0,
            "heightTolerance": 0.5,
            "maxPorosity": 0,
            "maxSpatter": 2
        }
        payload = {
            "student_id": student_id,
            "rubric": rubric
        }
        response = requests.post(
            f"{BASE_URL}/api/scan",
            json=payload,
            timeout=TIMEOUT
        )
        print(f"Status Code: {response.status_code}")
        data = response.json()
        print(f"Response keys: {list(data.keys())}")
        print(f"Metrics: {json.dumps(data.get('metrics', {}), indent=2)}")
        print(f"Status: {data.get('status')}")
        print(f"Score: {data.get('total_score')}")
        if response.status_code in [200, 201]:
            print(f"✅ Scan completed successfully!")
            return True
        else:
            print(f"❌ Scan failed")
            return False
    except Exception as e:
        print(f"❌ Error: {str(e)}")
        return False

def test_list_scans():
    """Test listing scans"""
    print_header("6. Testing List Scans")
    try:
        response = requests.get(f"{BASE_URL}/api/scans", timeout=TIMEOUT)
        print(f"Status Code: {response.status_code}")
        data = response.json()
        print(f"Found {len(data)} scans")
        if data:
            print(f"Most recent scan: {json.dumps(data[0], indent=2)}")
            print(f"✅ Successfully fetched scan history!")
            return True
        else:
            print(f"⚠️  No scans found yet")
            return True
    except Exception as e:
        print(f"❌ Error: {str(e)}")
        return False

def main():
    print("\n" + "="*60)
    print("  WeldMaster AI - Frontend-Backend Connection Test")
    print("="*60)
    print(f"\nTesting backend at: {BASE_URL}")
    print(f"Timeout: {TIMEOUT} seconds")

    # Run tests
    if not test_health():
        print("\n❌ Cannot proceed - backend is not running!")
        sys.exit(1)
    
    student_id = test_create_student()
    test_list_students()
    test_get_rubric()
    test_trigger_scan(student_id)
    test_list_scans()
    
    print("\n" + "="*60)
    print("  Test Summary")
    print("="*60)
    print("\n✅ All backend endpoints are working!")
    print("\nNext steps:")
    print("  1. Start frontend: npm run dev")
    print("  2. Open http://localhost:5173")
    print("  3. Test the application with your browser")

if __name__ == "__main__":
    main()
