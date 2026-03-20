import zmq
import time

def telemetry_publisher(q, port=5555):
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://*:{port}")

    print(f"[TELEM] publishing on tcp://*:{port}")

    while True:
        try:
            data = q.get()
            sock.send_pyobj(data)
        except Exception as e:
            print("[TELEM] error:", e)
            time.sleep(0.1)
