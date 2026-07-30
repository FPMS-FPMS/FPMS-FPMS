# FPMS Dashboard — production container.
#
# Runs the FastAPI backend serving the built React SPA. Deploy to any
# container host (AWS App Runner / ECS Fargate / Cloud Run / Fly.io /
# DigitalOcean App Platform).
#
# Build once with Node to compile the frontend, then discard Node in a
# slim runtime image.

# ---- Stage 1: build the React frontend ------------------------------------
FROM node:22-alpine AS frontend-build
WORKDIR /app
COPY frontend/package.json frontend/package-lock.json* ./
RUN npm ci --loglevel=error || npm install --loglevel=error
COPY frontend/ ./
RUN npm run build

# ---- Stage 2: Python runtime ----------------------------------------------
FROM python:3.12-slim
WORKDIR /app

# System deps for paramiko (crypto) + healthcheck curl
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential libffi-dev libssl-dev curl \
 && rm -rf /var/lib/apt/lists/*

COPY backend/requirements.txt ./backend/requirements.txt
RUN pip install --no-cache-dir -r backend/requirements.txt \
 && pip uninstall -y pywinpty pywebview || true

# Copy backend + the built frontend
COPY backend/ ./backend/
COPY --from=frontend-build /app/dist ./frontend/dist

ENV FPMS_BIND_HOST=0.0.0.0 \
    FPMS_BIND_PORT=8000 \
    FPMS_FRONTEND_DIST=/app/frontend/dist \
    PYTHONUNBUFFERED=1

EXPOSE 8000

HEALTHCHECK --interval=30s --timeout=5s --retries=3 \
  CMD curl -fsS http://localhost:8000/api/health || exit 1

CMD ["python", "-m", "uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "8000", "--proxy-headers", "--forwarded-allow-ips", "*"]
