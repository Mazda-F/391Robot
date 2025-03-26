import cv2
import numpy as np
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QWidget, QLabel, QPushButton, QVBoxLayout

class MjpegStreamThread(QThread):
    frame_received = pyqtSignal(QImage)
    connection_error = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.stream_url = 'http://192.168.4.1:129/stream'
        self._running = True

    def run(self):
        import urllib.request
        try:
            with urllib.request.urlopen(self.stream_url) as stream:
                data = b""
                while self._running:
                    chunk = stream.read(1024)
                    if not chunk:
                        break
                    data += chunk
                    a = data.find(b'\xff\xd8')  # JPEG start
                    b = data.find(b'\xff\xd9')  # JPEG end
                    if (a != -1 and b != -1 and b > a) or len(data) > 8000:
                        jpg = data[a:b+2]
                        data = data[b+2:]
                        img_array = np.frombuffer(jpg, dtype=np.uint8)
                        if img_array.size == 0:
                            continue
                        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                        if img is None:
                            continue
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        height, width, channels = img.shape
                        bytesPerLine = channels * width
                        qimage = QImage(img.data, width, height, bytesPerLine, QImage.Format_RGB888)
                        self.frame_received.emit(qimage)
        except Exception as e:
            error_msg = f"Connection Error"
            self.connection_error.emit(error_msg)

    def stop(self):
        self._running = False

class MjpegStreamWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(400)
        self.layout = QVBoxLayout(self)
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.image_label)

        self.retry_button = QPushButton("Retry", self)
        self.retry_button.clicked.connect(self.retry_connection)
        self.retry_button.hide()  # hide by default
        self.layout.addWidget(self.retry_button)

        self.start_stream_thread()

    def start_stream_thread(self):
        self.thread = MjpegStreamThread()
        self.thread.frame_received.connect(self.update_image)
        self.thread.connection_error.connect(self.handle_connection_error)
        self.thread.start()

    @pyqtSlot(QImage)
    def update_image(self, qimage):
        
        self.image_label.setText("")
        self.retry_button.hide()
        pixmap = QPixmap.fromImage(qimage)
        self.image_label.setPixmap(pixmap.scaled(self.image_label.size(),
                                                  Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation))

    @pyqtSlot(str)
    def handle_connection_error(self, error_msg):
        self.image_label.setText(error_msg)
        self.retry_button.show()

    def retry_connection(self):
        # stop current thread and start new one
        if self.thread.isRunning():
            self.thread.stop()
            self.thread.wait()
        self.retry_button.hide()
        self.image_label.setText("Retrying...")
        self.start_stream_thread()

    def closeEvent(self, event):
        if self.thread.isRunning():
            self.thread.stop()
            self.thread.wait()
        event.accept()
