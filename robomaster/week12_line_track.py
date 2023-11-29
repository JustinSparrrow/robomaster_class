import cv2
from robomaster import robot
import keyboard
import math
import time
from robomaster import vision


class PIDCtrl:
    def __init__(self, kp, ki, kd):
        self.__version__ = "0.0.1"
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.err = 0
        self.sum_err = 0
        self.last_err = 0

    def setErr(self, err):
        self.err = err
        self.sum_err += err
    @property
    def output(self):
        out = self.kp * self.err
        out += self.ki * self.sum_err
        out += self.kd * (self.err - self.last_err)
        self.last_err = self.err
        return out


class PointInfo:

    def __init__(self, x, y, theta, c):
        self._x = x
        self._y = y
        self._theta = theta
        self._c = c

    @property
    def pt(self):
        return int(self._x * 1280), int(self._y * 720)

    @property
    def color(self):
        return 255, 255, 255
    @property
    def distance(self):
        return math.sqrt(abs(self._x-0.5)*abs(self._x-0.5)+(0.5-self._y)*(0.5-self._y))

line = []

def on_detect_line(line_info):
    number = len(line_info)
    line.clear()
    for i in range(1,number):
        x, y, ceta, c = line_info[i]
        # print("x:{}, y:{}, ceta:{}, c:{}".format(x, y, ceta, c))
        line.append(PointInfo(x, y, ceta, c))


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")

    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
  
    ep_chassis=ep_robot.chassis
    #开启视频流
    ep_camera.start_video_stream(display=False)
    #订阅寻迹线识别信息
    result = ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)


    pid=PIDCtrl(40,0,3) #控制差速的PID控制器
    baseSpeed=40 #两侧轮子的基础速度
    difSpeed=0 #两侧轮子的差速
    quitCnt=0 #防止识别不稳定时程序意外退出
    while True:
        #检测到ESC键按下则退出循环
        if keyboard.is_pressed('q'):
            break
        #读一帧图像
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        #将寻迹线点信息的副本存入Line列表
        Line=line.copy()
        if len(Line)==0:#如果没有识别到寻迹线则增加quitCnt计数
            quitCnt+=1
        else:#如果识别到寻迹线则将quitCnt计数清0
            quitCnt=0
        if quitCnt>10:#如果连续10次识别不到寻迹线，则跳出主循环
            break
        min_distance=1.25#用于存放最小距离的变量
        min_err_x=0.5#用于存放最小距离点的x方向偏差
        #遍历所有的点
        for j in range(0, len(Line)):
            #使用OpenCV把点绘制在摄像头回传的图片上
            cv2.circle(img, Line[j].pt, 3, Line[j].color, -1)
            #寻找距离机器人最近的点，并记录它的x方向偏差
            # print(Line[j].distance,min_distance)
            if Line[j].distance<min_distance:
                min_distance=Line[j].distance
                min_err_x=Line[j]._x-0.75
                # print(min_err_x)
        #显示图片
        cv2.imshow("Line", img)
        cv2.waitKey(1)
        lSpeed=0#存放左轮速度的变量
        rSpeed=0#存放右轮速度的变量
        if min_err_x!=0.5:
            pid.setErr(min_err_x) #把偏差输入PID控制器
            difSpeed=pid.output #差速为PID控制器的输出
            # print(difSpeed)
            lSpeed=20 + difSpeed #左轮速度为基础速度+差速
            rSpeed=20 - difSpeed #右轮速度为基础速度-差速
        #写入左右轮速度
        ep_chassis.drive_wheels(w2=lSpeed,w3=rSpeed,w1=lSpeed,w4=rSpeed)
        # print(lSpeed,rSpeed)
        time.sleep(0.1)
    #退出循环后机器人停止
    ep_chassis.drive_wheels(0,0,0,0)
    cv2.destroyAllWindows()
    result = ep_vision.unsub_detect_info(name="line")
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()