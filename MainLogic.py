
#######################################################################################################################################################################
#                                                                      flow_v2 수정 내용                                                                              
# ball_bool 추가 (line  39,74,414~416)  
# detect_ball 추가 (line 326~381)
# & -> and (line 265)
# spd, time 추가 및 rate 조절 필요시 추가 (line 159~172)
# ball_position 한번에 하도록 (line 197,217)
# 노드 초기화 (line 44~45)
# 노드 동작 관리 (line 455~460)
#######################################################################################################################################################################


from typing import Literal
import math
import rospy
from std_msgs.msg import String, Bool, Int32, Float32
from geometry_msgs.msg import Point

class Golf_Flow:
    def __init__(self, golf_course = 'par 3'):
        # 이 부분은 사전에 정의된 거임. 한번 정하면 달라지지 않음
        self.mask_w_05, self.mask_h_05 = 200, 150 # int, int 공을 치고 나서 공을 추적할 때 공이 위치하고 있어야 할 영역
        self.middle_05 = 20 # int 좌우의 가운데라고 정의할 영역의 반폭
        self.center_05 = 20 # int 모든 영역의 정가운데라고 정의할 영역의 반폭(정사각형임 width == height == 2 * center_05
        self.walk_limit = 150 # int walk_limit 선 위에 공이 위치하면 앞으로 걸어감
        self.initial_head_angle = 43 # int 준비 자세 정하는 머리 각도
        self.actual_head_angle = 31 # int 실전 자세 정하는 머리 각도
        
        # 인식부 정의
        self.ball_position = None
        self.hole_position = None
        self.hole_dis = None
        self.robot_position = None
        self.bunker_bool = None
        self.bunker_dis = None
        self.ball_loc = None
        self.s_line_bool = None
        self.ball_bool = None
        # 동작부 정의
        self.field_angle = None
        self.head_angle = None
        
        # ROS 노드 초기화
        rospy.init_node('Golf_Flow', anonymous=True)
        

        # 동작부 publisher 정의
        self.pub_move_head_part = rospy.Publisher('/move_head_part', String, queue_size=1)
        self.pub_move_head_speed = rospy.Publisher('/move_head_speed', Int32, queue_size=1)
        self.pub_move_head_direction = rospy.Publisher('/move_head_direction', String, queue_size=1)
        self.pub_move_head_time = rospy.Publisher('/move_head_time', Float32, queue_size=1)
        self.pub_move_body_head_horizontal_angle = rospy.Publisher('/move_body_head_horizontal_angle', Int32, queue_size=1)
        self.pub_walk_forward_head_vertical_angle = rospy.Publisher('/walk_forward_head_vertical_angle', Int32, queue_size=1)
        self.pub_walk_forward_speed = rospy.Publisher('/walk_forward_speed', Int32, queue_size=1)
        self.pub_walk_forward_time = rospy.Publisher('/walk_forward_time', Float32, queue_size=1)
        self.pub_walk_backward_head_vertical_angle = rospy.Publisher('/walk_backward_head_vertical_angle', Int32, queue_size=1)
        self.pub_walk_backward_speed = rospy.Publisher('/walk_backward_speed', Int32, queue_size=1)
        self.pub_walk_backward_time = rospy.Publisher('/walk_backward_time', Float32, queue_size=1)
        self.pub_move_circle_direction = rospy.Publisher('/move_circle_direction', String, queue_size=1)
        self.pub_move_circle_angle = rospy.Publisher('/move_circle_angle', Int32, queue_size=1)
        self.pub_move_left_90_s_line_bool = rospy.Publisher('/move_left_90_s_line_bool', Bool, queue_size=1)
        self.pub_hit_ball_power = rospy.Publisher('/hit_ball_power', Int32, queue_size=1)
        self.pub_hit_ball_direction = rospy.Publisher('/hit_ball_power_direction', String, queue_size=1)
        # 인식부 subscriber 정의
        rospy.Subscriber("/ball_position", Point, callback=self.ball_position_func)
        rospy.Subscriber("/hole_position", Point, callback=self.hole_position_func)
        rospy.Subscriber("/hole_dis", Float32, callback=self.hole_dis_func)
        rospy.Subscriber("/robot_position", Bool, callback=self.robot_position_func)
        rospy.Subscriber("/bunker_bool", Bool, callback=self.bunker_func)
        rospy.Subscriber("/bunker_dis", Float32, callback=self.bunker_func)
        rospy.Subscriber("/ball_loc", String, callback=self.ball_loc_func)
        rospy.Subscriber("/s_line_bool", Bool, callback=self.s_line_bool_func)
        rospy.Subscriber("/ball_bool", Bool, callback=self.ball_bool_func)

        # 동작부 subscriber 정의
        rospy.Subscriber("/field_angle", Point, callback=self.field_angle_func)
        rospy.Subscriber("/head_angle", Point, callback=self.head_angle_func)
        
        '''
        # 이 부분은 인식부에서 받아와야할 변수 --> 밑에 함수로 def 해주삼 시리얼 부분만
        ball_position = (10,40) # tuple(y: int, x: int)
        hole_position = (10,40) # tuple(y: int, x: int)
        hole_dis = 100.0 # (float) meter
        robot_position = True # bool, 경기장 안에 있는지 여부
        bunker_bool, bunker_dis = True, 20.0 # bool, (float) meter
        ball_loc = 'light' # 'light', 'dark', 'out', 'bucker', 'hole'  light = light_green, dark = dark_green 공이 경기장 안에서 어느 위치에 있는지 알려줌
        s_line_bool = True # bool, 로봇, 공, 홀의 straight line 가능 여부
        hbr_angle = 120 # (int) hole to ball to robot 각도
        ball_bool = True # bool, 카메라 시야 안에 공이 들어와 있는지 여부

        # 이 부분은 동작부에서 받아와야할 변수 --> 밑에 함수로 def 해주삼 시리얼 부분만
        #x_field_angle, y_field_angle = field_angle() / return 형식 (2,3) # tuple(int,int) x축, y축 /동작에서 받아오기 로봇 자이로 x축, y축 각도 받아오는 함수
        #head_horizontal_angle, head_vertical_angle = head_angle() / return 형식 (100,30) # tuple(int,int) 좌우각도, 상하각도 /동작에서 받아오기 머리 상하, 좌우 각도 받아오는 함수
        
        # 이 부분은 동작부로 전해줘야할 움직이라는 함수 --> 밑에 함수로 def 해주삼 시리얼 부분만
        #move_head(part, speed, direction, time) 머리 각도를 바꾸는 함수, part = 좌우-'horizontal' or 상하-'vertical'인지 정해줌
        #move_body(head_horizontal_angle) 지금 위치한 머리각도 만큼 움직여서 머리와 몸이 바라보는 각도가 일치하도록 몸을 움직여주는 함수
        #walk_forward(head_vertical_angle, speed, time) 앞으로 걸어가는 함수/ 머리 각도에 따라서 걷는 속도를 다르게 함
        #walk_backward(head_vertical_angle, speed, time) 뒤로 걸어가는 함수/ 머리 각도에 따라서 걷는 속도를 다르게 함
        #move_circle(direction, angle) 방향과 각도를 지정해줘서 원 모양으로 움직이는 함수/ direction = 'left' or 'right', angle = 40: int
        #move_left_90(s_line) 왼쪽으로 90도 이동하는 함수
        #hit_ball(power, direction: Literal['left', 'right'] = 'left') 공을 결정된 힘으로 왼쪽이나 오른쪽으로 치는 함수
        '''

        if golf_course == 'par 3':
            # 아니면 그 전에 par 3임을 알려줄 수 있는 동작을 하게 시킴
            self.par_3_flow()
    
        elif golf_course == 'par 4':
            # 아니면 그 전에 par 4임을 알려줄 수 있는 동작을 하게 시킴
            self.par_4_flow()  
    
    def par_3_flow(self): # 파3 동작 방식
        while True:
            robot_position = self.robot_position
            x_field_angle, y_field_angle = self.field_angle.x, self.field_angle.y
            if not robot_position:
                # 삐 소리 내게 해도 좋을 듯
                break
            else:
                self.move_to_center()
                self.walk_to_ball(mode='initial')
                s_line_bool = self.s_line_bool

            
                
                if s_line_bool:
                    hole_dis, bunker_bool, bunker_dis = self.be_straight_line(s_line_bool)
                    robot_position = self.robot_position
                    if not robot_position:
                        # 삐 소리 내게 해도 좋을 듯
                        break
                    self.move_left_90(s_line_bool) # 만약에 경기장 각도가 공의 움직임에 영향을 많이 미친다면 field_angle을 이용해서 칠 위치를 정해야함. 90도 말고 다른 각도로 정해야함
                    robot_position = self.robot_position
                    if not robot_position:
                        # 삐 소리 내게 해도 좋을 듯
                        break
                else:
                    # 직선이 불가능한 겨우 도입
                    print('직선이 불가능한 겨우 도입')
                
                self.walk_to_ball(mode='actual')
                power = self.decide_power(hole_dis, bunker_bool, bunker_dis)
                self.hit_ball(power,'left')
                self.detect_ball()
                self.move_to_center()
                ball_loc = self.ball_loc
                if ball_loc == 'out':
                    # 삐 소리 내게 해도 좋을 듯
                    break
                elif ball_loc == 'hole':
                    # 세레모니 동작하기
                    break
    
    def par_4_flow(self):
        print('파 4 플로우 동작')

    # 머리와 몸이 공을 향해 움직이도록 하는 함수
    def move_to_center(self):
        speed = 20
        time = 0.5
        #rate = rospy.Rate(10) 작동시켜보고 필요하면 추가
        
        while True:
            head_horizontal_angle, head_vertical_angle = self.move_head_to_center(speed, time)
            self.move_body(head_horizontal_angle)
            ball_position = self.ball_position
            if 319 - self.middle_05 < ball_position.x < 319 + self.middle_05:
                head_horizontal_angle, head_vertical_angle = self.move_head_to_center(speed, time)
                break
           # rate.sleep()
        return head_horizontal_angle, head_vertical_angle
    
    # 머리를 중심으로 이동시키는 함수
    def move_head_to_center(self, speed, time):
        ball_position = self.ball_position
        
        while True:
            part = 'horizontal'
            x = ball_position.x
            if 0 < x < 640/3:
                direction = 'right'
                self.move_head(part, speed, direction, time) 
            elif 640/3 < x < 319 - self.middle_05:
                direction = 'right'
                self.move_head(part, speed, direction, time) 
            elif 640*2/3 > x > 319 + self.middle_05:
                direction = 'left'
                self.move_head(part, speed, direction, time) 
            elif 639 > x > 640*2/3:
                direction = 'left'
                self.move_head(part, speed, direction, time) 
            else:    
                break
            
            ball_position = self.ball_position
        
        while True:
            part = 'vertical'
            y = ball_position.y
            if 0 < y <= 480/3:
                direction = 'down'
                self.move_head(part, speed, direction, time) 
            elif 480/3 < y < 239 - self.center_05:
                direction = 'down'
                self.move_head(part, speed, direction, time) 
            elif 480*2/3 > y > 239 + self.center_05:
                direction = 'up'
                self.move_head(part, speed, direction, time) 
            elif 479 > y >= 480*2/3:
                direction = 'up'
                self.move_head(part, speed, direction, time) 
            else:
                break
            
            ball_position = self.ball_position
        
        head_horizontal_angle, head_vertical_angle = self.head_angle.x, self.head_angle.y
        return head_horizontal_angle, head_vertical_angle

    # 공의 위치로 이동하는 함수
    def walk_to_ball(self, mode: Literal['initial', 'actual'] = 'initial'): # mode: 'initial', 'actual', tan33가 대략 25/40, tan21가 대략 15/40 # initial = 준비자세, actual = 실전자세
        ball_position = self.ball_position # 공의 위치를 가져옴
        head_horizontal_angle, head_vertical_angle = self.head_angle.x, self.head_angle.y # 머리 각도 가져오기
        if mode == 'actual': # 실전자세로 가야할 때
            while True:
                if self.actual_head_angle - 1 < head_vertical_angle <= self.actual_head_angle: # 머리각도가 실전자세각도에 있는지 확인 후 break
                    ball_position = self.ball_position
                    break
                elif head_vertical_angle <= self.actual_head_angle - 1: # 머리각도가 실전자세각도보다 작은 경우 (너무 공과 가까운 경우)
                    self.walk_backward(head_vertical_angle, speed, time) # 뒤로 가기
                    head_horizontal_angle, head_vertical_angle = self.head_angle.x, self.head_angle.y # 너무 가까워서 공을 칠 수 있기에 몸을 머리각도와 평행하도록 움직이지 않는다
                    ball_position = self.ball_position
                else: # 머리각도가 실전자세각도보가 큰 경우 (공과 먼 경우)
                    self.walk_forward(head_vertical_angle, speed, time) # 앞으로 가기
                    '''
                    head_horizontal_angle, head_vertical_angle = self.move_head_to_center()
                    self.move_body(head_horizontal_angle)
                    '''
                    head_horizontal_angle, head_vertical_angle = self.move_to_center()
                    ball_position = self.ball_position
                
        elif mode == 'initial': # 준비자세로 가야할 때
            while True:
                if head_vertical_angle <= self.initial_head_angle: # 머리각도가 준비자세각도보다 작은 경우 (너무 가까운 경우)
                    self.walk_backward(head_vertical_angle, speed, time)
                    '''
                    head_horizontal_angle, head_vertical_angle = self.move_head_to_center()
                    self.move_body(head_horizontal_angle)
                    '''
                    head_horizontal_angle, head_vertical_angle = self.move_to_center()
                    ball_position = self.ball_position
                elif head_vertical_angle == self.initial_head_angle: # 머리각도가 준비자세각도인 경우
                    if 239 - self.center_05 < ball_position.y < 239 + self.center_05 and 319 - self.center_05 < ball_position.x < 319 + self.center_05: # 공이 정가운데에 있으면
                        break
                    else:
                        '''
                        head_horizontal_angle, head_vertical_angle = self.move_head_to_center()
                        self.move_body(head_horizontal_angle)
                        '''
                        head_horizontal_angle, head_vertical_angle = self.move_to_center()
                        ball_position = self.ball_position
                else: # 머리각도가 준비자세각도보다 큰 경우 (앞으로 가야함)
                    if 239 - self.center_05 < ball_position.y < 239 + self.center_05 and 319 - self.center_05 < ball_position.x < 319 + self.center_05: # 공이 정가운데에 있으면
                        self.walk_forward(head_vertical_angle, speed, time) # 앞으로 가기
                        ball_position = self.ball_position
                    elif 239 + self.center_05 <= ball_position.y < 389: # 공의 위치가 앞으로 가는 선을 안 넘었으면
                        self.walk_forward(head_vertical_angle, speed, time) # 계속 앞으로 간다
                        ball_position = self.ball_position
                    else: # 공의 위치가 앞으로 가는 선을 넘었으면
                        '''
                        head_horizontal_angle, head_vertical_angle = self.move_head_to_center()
                        self.move_body(head_horizontal_angle)
                        '''
                        head_horizontal_angle, head_vertical_angle = self.move_to_center() # 머리 움직여서 공이 정가운데에 있겠금함
                        ball_position = self.ball_position
        return ball_position, head_vertical_angle
    
    # 직선 상태를 유지하며 이동하는 함수
    def be_straight_line(self, s_line_bool:bool):
        if s_line_bool:
            angle = 90  
            while True:    
                ball_position = self.ball_position
                hole_position = self.hole_position
                if (319 - self.center_05 < ball_position.x < 319 + self.center_05 and
                        319 - self.center_05 < hole_position.x < 319 + self.center_05):
                    hole_dis = self.hole_dis
                    bunker_bool = self.bunker_bool
                    bunker_dis = self.bunker_dis
                    break
                elif ball_position.x < hole_position.x:
                    self.move_circle('left', angle)
                    self.move_to_center()
                    self.walk_to_ball('initial')
                elif ball_position.x > hole_position.x:
                    self.move_circle('right', angle)
                    self.move_to_center()
                    self.walk_to_ball('initial')
        else:
            hole_dis = None
            bunker_bool, bunker_dis = False, 0.0
        
        return hole_dis, bunker_bool, bunker_dis

    def move_left_90(self, s_line_bool):
        self.pub_move_left_90_s_line_bool.publish(s_line_bool)

    # 칠 파워 결정
    def decide_power(self, hole_dis, bunker_bool, bunker_dis) -> int:
        cf, cb, c = 1.2, 1.5, 0.8
        ball_loc = self.ball_loc
        if bunker_bool:
            power = cf * (hole_dis - bunker_dis) + cb * bunker_dis
            if ball_loc == 'bunker':
                power += c
        else:
            power = cf * hole_dis
        return power
    
    def hit_ball(self, power, direction: Literal['left', 'right'] = 'left'):
        self.pub_hit_ball_power.publish(power)
        self.pub_hit_ball_direction.publish(direction)

    def detect_ball(self):
        speed = 25
        time_interval = 0.1
        max_attempts = 20
        steps_back = 3
        walk_speed = 10
        walk_time = 1.0
        sleep_duration = 0.1
        
        def update_ball_position():
            rospy.sleep(sleep_duration)
            return self.ball_position

        def centralize_ball():
            ball_position = update_ball_position()
            while True:
                if ball_position.x > 519:
                    self.move_head('horizontal', speed, 'left', time_interval)
                elif ball_position.x < 119:
                    self.move_head('horizontal', speed, 'right', time_interval)
                elif ball_position.y > 389:
                    self.move_head('vertical', speed, 'up', time_interval)
                elif ball_position.y < 89:
                    self.move_head('vertical', speed, 'down', time_interval)
                else:
                    break
                ball_position = update_ball_position()

        attempts = 0
        while attempts < max_attempts:
            self.ball_bool = self.ball_bool
            if self.ball_bool:
                centralize_ball()
                return self.head_angle.x, self.head_angle.y
            
            for direction in ['left', 'right']:
                self.move_head('horizontal', speed, direction, time_interval)
                rospy.sleep(sleep_duration)
                if self.ball_bool:
                    centralize_ball()
                    return self.head_angle.x, self.head_angle.y
            
            for direction in ['up', 'down']:
                self.move_head('vertical', speed, direction, time_interval)
                rospy.sleep(sleep_duration)
                if self.ball_bool:
                    centralize_ball()
                    return self.head_angle.x, self.head_angle.y

            attempts += 1
            if attempts % (max_attempts // steps_back) == 0:
                for _ in range(steps_back):
                    self.walk_backward(self.head_angle.y, walk_speed, walk_time)
                    rospy.sleep(walk_time)
    
        return None

    # 인식부
    def ball_position_func(self, data): #return --> (10,40) # tuple(y: int, x: int)
        if isinstance(data, Point):
            self.ball_position = data # x좌표 = ball_position.x, y좌표 = ball_position.y

    def hole_position_func(self, data): #return --> (10,40) # tuple(y: int, x: int)
        if isinstance(data, Point):
            self.hole_position = data # x좌표 = hole_position.x, y좌표 = hole_position.y

    def hole_dis_func(self, data): #return --> 100.0 # (float) meter
        if isinstance(data, Float32):
            self.hole_dis = data.data
            
    def robot_position_func(self, data): #return --> True # bool, 경기장 안에 있는지 여부
        if isinstance(data, Bool):
            self.robot_position = data.data
            
    def bunker_func(self, data): #return --> bunker_bool, bunker_dis = True, 20.0 # bool, (float) meter
        if isinstance(data, Bool):
            self.bunker_bool = data.data
        elif isinstance(data, Float32):
            self.bunker_dis = data.data
    
    def ball_loc_func(self, data): #return --> 'light' # 'light', 'dark', 'out', 'bucker', 'hole'  light = light_green, dark = dark_green 공이 경기장 안에서 어느 위치에 있는지 알려줌
        if isinstance(data, String):
            self.ball_loc = data.data
            
    def s_line_bool_func(self, data): #return --> True # bool, 로봇, 공, 홀의 straight line 가능 여부
        if isinstance(data, Bool):
            self.s_line_bool = data.data

    def ball_bool_func(self, data):
        if isinstance(data, Bool):
            self.ball_bool = data.data
            
    #def hbr_angle_func(self): #return --> 120 # int
    
    
    #동작부
    # 이 부분은 동작부에서 받아와야할 변수
    def field_angle_func(self, data): #return --> x_field_angle, y_field_angle /형식 (2,3) # tuple(int,int) x축, y축 /동작에서 받아오기 로봇 자이로 x축, y축 각도 받아오는 함수
        if isinstance(data, Point):
            self.field_angle = data
            
    def head_angle_func(self, data): #return --> head_horizontal_angle, head_vertical_angle /형식 (100,30) # tuple(int,int) 좌우각도, 상하각도 /동작에서 받아오기 머리 상하, 좌우 각도 받아오는 함수
        if isinstance(data, Point):
            self.head_angle = data
            
    # 이 부분은 동작부로 전해줘야할 움직이라는 함수 
    def move_head(self, part, speed, direction, time): #머리 각도를 바꾸는 함수, part = 상하 or 좌우인지 정해줌
        self.pub_move_head_part.publish(part)
        self.pub_move_head_speed.publish(speed)
        self.pub_move_head_direction.publish(direction)
        self.pub_move_head_time.publish(time)
        
    def move_body(self, head_horizontal_angle): #지금 위치한 머리각도 만큼 움직여서 머리와 몸이 바라보는 각도가 일치하도록 몸을 움직여주는 함수
        self.pub_move_body_head_horizontal_angle.publish(head_horizontal_angle)
        
    def walk_forward(self, head_vertical_angle, speed, time): #앞으로 걸어가는 함수/ 머리 각도에 따라서 걷는 속도를 다르게 함
        self.pub_walk_forward_head_vertical_angle.publish(head_vertical_angle)
        self.pub_walk_forward_speed.publish(speed)
        self.pub_walk_forward_time.publish(time)
        
    def walk_backward(self, head_vertical_angle, speed, time): #뒤로 걸어가는 함수/ 머리 각도에 따라서 걷는 속도를 다르게 함
        self.pub_walk_backward_head_vertical_angle.publish(head_vertical_angle)
        self.pub_walk_backward_speed.publish(speed)
        self.pub_walk_backward_time.publish(time)
        
    def move_circle(self, direction, angle): #방향과 각도를 지정해줘서 원 모양으로 움직이는 함수/ direction = 'left' or 'right', angle = 40: int
        self.pub_move_circle_direction.publish(direction)
        self.pub_move_circle_angle.publish(angle)
        
    #def move_left_90(self, s_line): # 왼쪽으로 90도 이동하는 함수
    
    #def hit_ball(self, power, direction: Literal['left', 'right'] = 'left'): # 공을 결정된 힘으로 왼쪽이나 오른쪽으로 치는 함수
    
if __name__ == '__main__':
    try:
        golf_flow = Golf_Flow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
