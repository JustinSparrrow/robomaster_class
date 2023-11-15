import time
from robomaster import robot
from robomaster import camera


class MarkerInfo:

    def __init__(self, x, y, w, h, info) -> None:
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info

    @property
    def pt1(self):
        return int((self._x-self._w/2)*1280), int((self._y-self._h/2)*720)

    @property
    def pt2(self):
        


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_camera = ep_robot.camera

    # 显示10s图传
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)
    time.sleep(10)
    ep_camera.stop_video_stream()

    ep_robot.close()
