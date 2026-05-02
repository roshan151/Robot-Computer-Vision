"""ASGI entry for the vision API (YOLO). Run from repo root: uvicorn Vision.app:app --host 0.0.0.0 --port 8080"""

from fastapi import FastAPI

from .service import router

app = FastAPI(title="Robot vision")
app.include_router(router)
