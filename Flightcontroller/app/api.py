from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from .config import settings
from .state import TelemetryState
from .telemetry_service import TelemetryService

telemetry_state = TelemetryState()
telemetry_service = TelemetryService(settings, telemetry_state)


@asynccontextmanager
async def lifespan(_: FastAPI):
    telemetry_service.start()
    try:
        yield
    finally:
        telemetry_service.stop()


app = FastAPI(
    title="KRTI VTOL Flight Controller Bridge",
    version="0.1.0",
    lifespan=lifespan,
)

# Untuk pengembangan lokal. Saat produksi, ganti "*" dengan origin BaseStation.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["GET"],
    allow_headers=["*"],
)


@app.get("/health")
def health() -> dict:
    snapshot = telemetry_state.snapshot()
    connection = snapshot["connection"]
    return {
        "service": "flightcontroller",
        "status": "ok" if connection["connected"] else "degraded",
        "connected": connection["connected"],
        "port": connection["port"],
        "error": connection["error"],
    }


@app.get("/api/telemetry/latest")
def latest_telemetry() -> dict:
    return telemetry_state.snapshot()


@app.get("/api/status-text")
def status_text() -> list[dict]:
    return telemetry_state.snapshot()["status_text"]


@app.websocket("/ws/telemetry")
async def telemetry_websocket(websocket: WebSocket) -> None:
    await websocket.accept()
    interval_s = 1.0 / max(settings.websocket_rate_hz, 0.1)

    try:
        while True:
            await websocket.send_json(telemetry_state.snapshot())
            await asyncio.sleep(interval_s)
    except WebSocketDisconnect:
        pass
