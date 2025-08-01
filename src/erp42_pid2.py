#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from erp42_serial2.msg import ERP_STATUS, ERP_SET, ERP_CMD
import time
import numpy as np  # np.log를 사용하기 위해 numpy를 임포트

class ERP42Controller:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('erp42_controller')

        # ERP_SET 메시지 구독
        rospy.Subscriber('/erp42_set', ERP_SET, self.erp_set_callback)
        # ERP_STATUS 메시지 구독
        rospy.Subscriber('/erp42_status', ERP_STATUS, self.erp_status_callback)

        # ERP_CMD 메시지 발행
        self.erp_cmd_pub = rospy.Publisher('/erp42_cmd', ERP_CMD, queue_size=1)
        self.error_pub = rospy.Publisher('/erp42_error', Float64, queue_size=1)

        # PID 제어 파라미터 초기화
        self.kp = rospy.get_param("~KP", 1.16704)
        self.ki = rospy.get_param("~KI", 18.056)
        self.kd = rospy.get_param("~KD", 1.5)
        self.anti_windup_guard = rospy.get_param("~windup_guard", 40.0)
        self.control_limit = rospy.get_param("~control_limit", 250)
        self.error_rate = rospy.get_param("~error_rate", 0.1)
        self.reset_threshold = rospy.get_param("~reset_threshold", 10)
        # 목표값과 현재값 초기화
        self.desired_speed = 0
        self.desired_steer = 0
        self.desired_brake = 0
        self.desired_gear = 0
        self.current_speed = 0

        # PID 제어를 위한 이전 오차 및 적분 오차 초기화
        self.prev_error = 0
        self.error_i = 0

        # 자동 모드 초기화
        self.auto_mode = False

        # 제어 루프 주기 설정 (20 Hz)
        hz = 20
        self.dt = 1 / hz
        self.rate = rospy.Rate(hz)
        self.control_loop()

    def erp_set_callback(self, msg:ERP_SET):
        # set_state 토픽에서 목표 속도, 조향각, 브레이크, 기어 설정
        self.desired_speed = msg.set_speed * 10
        self.desired_steer = msg.set_steer
        self.desired_brake = msg.set_brake
        self.desired_gear = msg.set_gear
        rospy.loginfo(self.desired_steer)
        
    def erp_status_callback(self, msg):
        # 현재 속도 수신 및 자동 모드 설정
        self.current_speed = msg.status_speed
        self.auto_mode = msg.status_AorM == 1  # 자동 모드: 1, 수동 모드: 0

    def get_scaling_factor(self, target):
        """ 목표 속도에 따른 로그 및 2차 방정식 기반의 스케일링 값을 계산하는 함수 """
        if 50 < target <= 445:               # 로그함수는 0보다 큰 값에서 정의되며, target 값이 1보다 작아지면 목표속도보다 느려지거나 음수가 나와 full braking 할 수 있음
            y = (-1.1285 * np.log(target)   # 로그 함수
                 - 1.752e-5 * target ** 2   # 2차 함수
                 + 0.0124 * target          # 1차 항
                 + 5.8334)                  # 상수항
            y = y * 0.94
        elif target <= 50:
            y = 1.65
        else:
            y = 1  # target이 0 이하 또는 445 초과일 때 y = 1 반환
        
        return y  # <-- 스케일링 값 조정

    def calculate_pid(self, target, current, disired_upper, desired_lower):
        # 목표 속도에 따른 스케일링 값 적용
        scaling_factor = self.get_scaling_factor(target)
        scaled_target = target * scaling_factor


        # PID 제어 출력 계산
        if desired_lower < current < disired_upper:
            # 오차범위 안에 들면 스케일링 안함
            error = target - current
        else: 
            error = scaled_target - current

        #rospy.loginfo(f"Error: {error}, Scaling Factor: {scaling_factor}")
        self.error_pub.publish(error)
        self.error_i += error * self.dt  # 적분 오차 계산
        error_d = (error - self.prev_error) / self.dt  # 미분 오차 계산

        # Anti wind-up (적분 오차 제한)
        if self.error_i < -self.anti_windup_guard:
            self.error_i = -self.anti_windup_guard
        elif self.error_i > self.anti_windup_guard:
            self.error_i = self.anti_windup_guard

        # PID 제어 출력 계산
        pid_out = self.kp * error + self.ki * self.error_i + self.kd * error_d
        self.prev_error = error  # 이전 오차 업데이트

        # PID 제어 출력 값을 비율에 따라 조정 및 제한
        pid_out = np.clip(pid_out, -150, self.control_limit)
        # if pid_out > self.control_limit:
        #     pid_out = self.control_limit
        # elif pid_out < -150:
        #     pid_out = -150

        # 가속 및 브레이크 명령 계산
        if pid_out > 0:
            if desired_lower < current < disired_upper:
                accel_cmd = np.clip(pid_out, 0, disired_upper)
            else:
                accel_cmd = min(int(pid_out), 255)
            brake_cmd = 0
        
        else:
            accel_cmd = 0
            brake_cmd = min(int(abs(pid_out)), 150)

        return accel_cmd, brake_cmd

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.auto_mode == 1:
                # 속도 제어
                desired_upper = (1 + self.error_rate) * self.desired_speed
                desired_lower = (1 - self.error_rate) * self.desired_speed

                accel_cmd, brake_cmd = self.calculate_pid(self.desired_speed, self.current_speed, desired_upper, desired_lower)

                # 속도 오차가 일정 임계값을 초과하면 적분 오차 초기화
                if abs(self.desired_speed - self.current_speed) > self.reset_threshold:
                    self.error_i = 0

                # ERP_CMD 메시지 생성 및 발행
                erp_cmd_msg = ERP_CMD()
                erp_cmd_msg.cmd_gear = self.desired_gear
                erp_cmd_msg.cmd_speed = int(accel_cmd)
                erp_cmd_msg.cmd_steer = self.desired_steer
                erp_cmd_msg.cmd_brake = brake_cmd
                rospy.loginfo(erp_cmd_msg)
                self.erp_cmd_pub.publish(erp_cmd_msg)

            # 루프 주기 유지 (20 Hz)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ERP42Controller()
    except rospy.ROSInterruptException:
        pass
