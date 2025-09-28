import cv2

class CameraSensor:
    def __init__(self, camera_id=0):
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera {camera_id} could not be opened.")

    def capture_image(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError(f"Failed to capture image from camera {self.camera_id}")
        return frame

    def save_image(self, frame, path):
        cv2.imwrite(path, frame)

    def release(self):
        self.cap.release()
