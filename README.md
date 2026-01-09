# WeldMaster AI - Desktop Training Platform

**AI-powered weld defect detection with desktop model training and RDK X5 edge deployment**

## Quick Start

### Prerequisites
- **Node.js 18+** (for React UI)
- **Python 3.10+** (for Django Brain)

### Run the Desktop Application

```bash
# Install dependencies
npm install

# Start the full stack (Django + React)
npm run start
```

**This starts:**
- 🧠 **Django Brain** API on `http://localhost:8000`
- 🎨 **React UI** on `http://localhost:3002`

**Open your browser** to `http://localhost:3002` and start using the app!

---

## Architecture

### Desktop Brain (Current Setup)
- **Django 6.0** REST API with DRF
- **SQLite** database for students/scans/training jobs
- **Celery** (configured, not yet running) for background tasks
- **Python .venv** located in `desktop_server/.venv/`

### Frontend
- **React 19** + **TypeScript** + **Vite**
- **Recharts** for data visualization
- **Lucide React** icons

### RDK X5 Edge (Future Integration)
- Side-channel architecture ready
- RDK serves MJPEG stream directly to browser
- Django handles persistence, RDK handles live capture

---

## Development

### Backend (Django Brain)
```bash
cd desktop_server
.venv/Scripts/python manage.py runserver 0.0.0.0:8000
```

### Frontend (React UI)
```bash
npm run dev
```

### Database Migrations
```bash
cd desktop_server
.venv/Scripts/python manage.py migrate
.venv/Scripts/python manage.py createsuperuser  # optional
```

---

## Project Structure

```
desktop_server/          # Django Brain API
  ├── core/             # Main app (models, views, serializers)
  ├── weldmaster_brain/ # Django project settings
  └── .venv/            # Python virtual environment
rdk_agent/              # RDK X5 edge node (future)
components/             # React UI components
services/               # API client services
```

---

## Documentation

- [SETUP_GUIDE.md](SETUP_GUIDE.md) - Complete setup and configuration
- [START_HERE.md](START_HERE.md) - Desktop training overview
- [DESKTOP_MODEL_TRAINING_STRATEGY.md](DESKTOP_MODEL_TRAINING_STRATEGY.md) - Training architecture
- [RDK_STEREO_CAMERA_SPEC.md](RDK_STEREO_CAMERA_SPEC.md) - Hardware specs (future)
