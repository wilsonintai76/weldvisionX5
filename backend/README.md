# backend/

This folder is **not** the application backend server.

It currently contains:

- (Optional) Local artifacts/logs created during development (e.g. `weld_data.db`, `weld_evaluator.log`).
- Local artifacts/logs created during development (e.g. `weld_data.db`, `weld_evaluator.log`).

## What runs the backend API?

The backend API is Django and lives in `desktop_server/`.

- Start Django: `npm run backend`
- Start full stack (Django + React): `npm run start`

## Why keep this folder?

Historically this repo stored the Python environment here.

The Python virtual environment is now located at `desktop_server/.venv/` (see `package.json`).
