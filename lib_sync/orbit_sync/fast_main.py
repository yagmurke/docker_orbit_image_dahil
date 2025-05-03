from fastapi import FastAPI, WebSocket, WebSocketDisconnect, BackgroundTasks
from typing import List
import asyncio
from background_task import process_input_stream

app = FastAPI()

# Store active WebSocket connections
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            await connection.send_text(message)

manager = ConnectionManager()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """Handles WebSocket connections and messages"""
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            await manager.broadcast(f"Message from client: {data}")
    except WebSocketDisconnect:
        manager.disconnect(websocket)

@app.get("/start-stream")
async def start_stream(background_tasks: BackgroundTasks):
    """Starts background input stream processing"""
    background_tasks.add_task(process_input_stream)
    return {"message": "Background stream processing started"}
