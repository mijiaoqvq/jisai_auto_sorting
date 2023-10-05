import roslibpy
import asyncio
import websockets
from datetime import datetime

ros = roslibpy.Ros("127.0.0.1", 9090)
ros.run()
topic = roslibpy.Topic(ros, "/direction_ctl", "geometry_msgs/Twist")

async def handler(websocket):
    while True:
        data = await websocket.recv()
        reply = f"Data received as \"{data}\".  time: {datetime.now()}"
        print(reply)
        d = {
            "linear": {
                "x": 0,
                "y": 0,
                "z": 0
            },
            "angular": {
                "x": 0,
                "y": 0,
                "z": 0
            }
        }
        if data == "up":
            d["linear"]["x"] = 1
        if data == "down":
            d["linear"]["x"] = -1
        if data == "left":
            d["linear"]["y"] = 1
        if data == "right":
            d["linear"]["y"] = -1
        if data == "lt":
            d["angular"]["z"] = 1
        if data == "rt":
            d["angular"]["z"] = -1
        msg = roslibpy.Message(d)
        topic.publish(msg)


async def main():
    async with websockets.serve(handler, "0.0.0.0", 9999):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    while True:
        try:
            asyncio.run(main())
        except:
            pass
