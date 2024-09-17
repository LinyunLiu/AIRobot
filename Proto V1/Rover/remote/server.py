import cv2
from flask import Flask, Response, render_template, request
from picamera2 import Picamera2
import controller

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

app = Flask(__name__)
app = Flask(__name__, static_folder='templates')

def generate_frames():
    # camera = cv2.VideoCapture('/dev/video0')  # Use the correct device path for your webcam
    while True:
        frame = picam2.capture_array()
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/webcam')
def video_feed():
    # Video streaming route
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
@app.route('/')
def index():
    url = "http://10.18.15.53:3000"
    # Route to serve the HTML file
    return render_template('index.html', url=url)

@app.route('/command')
def command():
    c = request.args.get('c')
    match (c):
        case 'w':
            controller.forward()
        case 's':
            controller.reverse()
        case 'a':
            controller.turn_left(rate=0.15)
        case 'd':
            controller.turn_right(rate=0.15)
        case 'q':
            controller.spin_left()
        case 'e':
            controller.spin_right()
        case 'x':
            controller.stop()
            
    return f'command: {c}'


if __name__ == "__main__":
    app.run(host='0.0.0.0', port=3000)
