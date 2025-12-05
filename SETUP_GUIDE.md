# Frontend-Backend Connection Setup

## Overview
The WeldMaster AI Evaluation system now has a fully integrated frontend-backend architecture:

- **Frontend**: React + TypeScript (Vite) on `http://localhost:5173`
- **Backend**: Flask API on `http://localhost:5000`
- **Connection**: Real HTTP API calls (no mock data)

---

## Quick Start

### Prerequisites
- Node.js 18+ (for frontend)
- Python 3.8+ (for backend)

### 1. Backend Setup

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run backend server
python app.py
```

Backend will start on: **http://localhost:5000**

Health check endpoint: `GET http://localhost:5000/api/health`

### 2. Frontend Setup

```bash
# Navigate to project root (in new terminal)
cd d:\WeldMaster AI Evaluation

# Install dependencies
npm install

# Run development server
npm run dev
```

Frontend will start on: **http://localhost:5173**

---

## API Endpoints

### Students Management
- `GET /api/students` - List all students
- `POST /api/students` - Create new student
- `PUT /api/students/<id>` - Update student
- `DELETE /api/students/<id>` - Delete student

### Scanning
- `POST /api/scan` - Trigger welding evaluation scan
- `GET /api/scans` - Fetch scan history

### Rubric Configuration
- `GET /api/rubric` - Get current rubric
- `POST /api/rubric` - Save rubric configuration

### Calibration
- `POST /api/calibrate` - Run camera calibration
- `POST /api/calibrate/save` - Save calibration data

### System Health
- `GET /api/health` - Check backend status (ROS2, camera availability)

---

## Configuration

### Environment Variables

Create a `.env` file in the project root:

```env
VITE_API_URL=http://localhost:5000
```

This URL is used by the frontend to communicate with the backend.

---

## Troubleshooting

### Frontend Can't Connect to Backend

1. Verify backend is running:
   ```bash
   curl http://localhost:5000/api/health
   ```

2. Check `VITE_API_URL` in `.env`:
   ```env
   VITE_API_URL=http://localhost:5000
   ```

3. Clear browser cache and restart dev server:
   ```bash
   npm run dev
   ```

### Database Issues

If you see database errors, delete the old database and restart:

```bash
# In backend directory
rm weld_data.db
python app.py
```

This will auto-create a fresh database.

### Port Already in Use

Change the backend port in `backend/app.py`:

```python
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)  # Change 5000 to 5001
```

Then update `.env`:
```env
VITE_API_URL=http://localhost:5001
```

---

## Data Flow

1. **Frontend (React)** → Makes API calls via `apiService.ts`
2. **Backend (Flask)** → Processes requests, validates data
3. **Database (SQLite)** → Persists student/scan data
4. **Vision Engine** → Processes welding images (requires camera or mock data)
5. **Response** → Returns evaluation results to frontend

---

## Deployment Notes

### For RDK X5 Hardware
- Ensure ROS2 is installed on RDK X5
- Update `VITE_API_URL` to point to RDK X5's IP address
- Run backend on RDK X5 with `python app.py`

### Production Deployment
- Use WSGI server (Gunicorn) instead of Flask dev server:
  ```bash
  pip install gunicorn
  gunicorn -w 4 -b 0.0.0.0:5000 app:app
  ```
- Disable CORS in production or configure it properly
- Use environment-specific `.env` files
- Set `debug=False` in Flask app

---

## File Structure

```
d:\WeldMaster AI Evaluation\
├── .env                          # Environment config
├── .env.local                    # Local overrides
├── services/
│   ├── apiService.ts            # ✅ NEW: Real API calls
│   └── mockApiService.ts        # Old: Mock data (deprecated)
├── App.tsx                       # ✅ UPDATED: Uses apiService
├── backend/
│   ├── app.py                   # ✅ UPDATED: New endpoints
│   ├── requirements.txt
│   ├── database/
│   │   └── models.py
│   └── vision/
│       ├── calibration.py
│       └── evaluator.py
└── ... (other files)
```

---

## Changes Made

### Frontend
- ✅ Created `services/apiService.ts` with real HTTP calls
- ✅ Updated `App.tsx` to import from `apiService` instead of `mockApiService`
- ✅ Added environment variable support for API URL

### Backend
- ✅ Added `PUT /api/students/<id>` endpoint
- ✅ Added `DELETE /api/students/<id>` endpoint
- ✅ Added `GET /api/scans` endpoint
- ✅ Enhanced `POST /api/scan` with full metrics persistence
- ✅ Added `GET /api/rubric` endpoint
- ✅ Added `POST /api/rubric` endpoint
- ✅ Added `POST /api/calibrate` endpoint
- ✅ Added `POST /api/calibrate/save` endpoint
- ✅ Added `GET /api/health` endpoint for monitoring

---

## Testing the Connection

### 1. Test Backend Health
```bash
curl http://localhost:5000/api/health
```

Expected response:
```json
{
  "status": "ok",
  "ros2_available": false,
  "camera_frame": false,
  "camera_depth": false
}
```

### 2. Create a Student (Backend)
```bash
curl -X POST http://localhost:5000/api/students \
  -H "Content-Type: application/json" \
  -d '{"name":"John Doe","student_id":"S001","class_name":"Welding 101","level":"Novice"}'
```

### 3. List Students (from Frontend)
Navigate to `http://localhost:5173` → Students tab → Should see your student

### 4. Trigger a Scan (from Frontend)
Select a student → Scanner tab → Click scan button → Results appear in History tab

---

## Next Steps

1. ✅ Backend running on port 5000
2. ✅ Frontend running on port 5173
3. ✅ All endpoints implemented
4. **Action**: Open http://localhost:5173 in browser and test the system
5. **Validation**: Run the deployment on RDK X5 hardware
