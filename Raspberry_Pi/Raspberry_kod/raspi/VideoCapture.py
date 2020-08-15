import cv2
import numpy as np

VID_SCALE_FACTOR = 0.6 #percentage

class VideoCapture:
    def __init__(self, video_source):
        # Open the video source
        self.video = cv2.VideoCapture(video_source)
        if not self.video.isOpened():
            raise ValueError("Unable to open video source, check camera port", video_source)
        # Get video source width and height
        self.width = self.video.get(cv2.CAP_PROP_FRAME_WIDTH) * VID_SCALE_FACTOR
        self.height = self.video.get(cv2.CAP_PROP_FRAME_HEIGHT) * VID_SCALE_FACTOR

    def Get_Frame(self):
        if self.video.isOpened():
            _, self.frame = self.video.read()
            self.frame = cv2.resize(self.frame, (int(self.width), int(self.height)), interpolation=cv2.INTER_AREA)
            self.frame = cv2.flip(self.frame, 1)
            self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            return self.frame
        else:
            return None

    # Release the video source when the object is destroyed
    def __del__(self):
        if self.video.isOpened():
            self.video.release()
