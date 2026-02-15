# 这是一个纯 Python 的取景器，用于调试摄像头位置
import cv2
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import time

# 设置分辨率 (必须和 C++ 代码一致，否则视野不同)
WIDTH = 320
HEIGHT = 240

class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.endswith('.mjpg'):
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            while True:
                try:
                    rc, img = capture.read()
                    if not rc: continue
                    
                    # 画个中心十字架，帮你对准
                    cv2.line(img, (int(WIDTH/2), int(HEIGHT/2)-20), (int(WIDTH/2), int(HEIGHT/2)+20), (0,255,0), 2)
                    cv2.line(img, (int(WIDTH/2)-20, int(HEIGHT/2)), (int(WIDTH/2)+20, int(HEIGHT/2)), (0,255,0), 2)
                    
                    imgRGB = cv2.imencode('.jpg', img)[1].tobytes()
                    self.wfile.write(b"--jpgboundary\r\n")
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(imgRGB)))
                    self.end_headers()
                    self.wfile.write(imgRGB)
                except:
                    break
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html><head></head><body><img src="/cam.mjpg"/></body></html>')

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

# 1. 打开摄像头
capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

if not capture.isOpened():
    print("❌ 摄像头打不开！")
else:
    print(f"✅ 摄像头已就绪 ({WIDTH}x{HEIGHT})")
    print("📡 请在电脑浏览器打开: http://<香橙派IP>:8888")

# 2. 开启网页服务器
try:
    server = ThreadedHTTPServer(('0.0.0.0', 8888), CamHandler)
    server.serve_forever()
except KeyboardInterrupt:
    capture.release()
    server.socket.close()
