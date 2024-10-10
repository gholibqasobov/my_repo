import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
from pynput import keyboard
import threading



class TeleopKeyboard:
    def __init__(self) -> None:
        # Time count
        self.t = 0
        self.small_dt = 0.01
        self.dt = 0.5

        # Initial position and yaw
        self.px0 = 0
        self.py0 = 0
        self.yaw0 = 0

        # Motion and Rotation velocity
        self.lin_vel = 0.3
        self.rot_vel = 0.5

        # Create a sport client
        self.client = SportClient()  
        self.client.SetTimeout(10.0)  # 3.0
        self.client.Init()

        self.action = False
        self.standing = True  # Assume robot starts in standing position
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)    
    
    def GetInitState(self, robot_state: SportModeState_):
        self.px0 = robot_state.position[0]
        self.py0 = robot_state.position[1]
        self.yaw0 = robot_state.imu_state.rpy[2]

    def MoveForward(self):
        while self.action:
            self.client.Move(self.lin_vel, 0, 0)  # Move forward at 0.3 m/s
            time.sleep(self.small_dt)
    
    def MoveBackward(self):
        while self.action:
            self.client.Move(-self.lin_vel, 0, 0)  # Move backward at 0.3 m/s
            time.sleep(self.small_dt)
    
    def MoveLeft(self):
        while self.action:
            self.client.Move(0, self.lin_vel, 0)  # Move left at 0.3 m/s
            time.sleep(self.small_dt)
    
    def MoveRight(self):
        while self.action:
            self.client.Move(0, -self.lin_vel, 0)  # Move right at 0.3 m/s
            time.sleep(self.small_dt)

    def turnRight(self):
        while self.action:
            self.client.Move(0, 0, -self.rot_vel)  # Turn clockwise rotation
            time.sleep(self.small_dt)
    
    def turnLeft(self):
        while self.action:
            self.client.Move(0, 0, self.rot_vel)  # Turn left counterclockwise rotation
            time.sleep(self.small_dt)
    
    def ShakeHand(self):
        while self.action:
            self.client.Hello()  # Wave Hello with front right leg
            time.sleep(self.small_dt)
            self.client.BalanceStand()
            time.sleep(self.dt)

    def JumpForward(self): 
        while self.action:
            self.client.FrontJump()  # Jump Forward
            time.sleep(2)
            self.client.StandUp()
            time.sleep(self.dt)
            self.standing = True
            self.client.BalanceStand()
            time.sleep(self.dt)

    def Love(self):
        while self.action:
            self.client.Heart()  # Show heart with front legs
            time.sleep(self.small_dt)
            self.client.BalanceStand()
            time.sleep(self.dt)

    


    def on_press(self, key):
        try:
            if key.char and key.char.lower() == 'w':
                if not self.action:
                    self.action = True
                    self.thread = threading.Thread(target=self.MoveForward)
                    self.thread.start()
            elif key.char and key.char.lower() == 's':
                if not self.action:
                    self.action = True
                    self.thread = threading.Thread(target=self.MoveBackward)
                    self.thread.start()
            elif key.char and key.char.lower() == 'd':
                if not self.action:
                    self.action = True
                    self.thread = threading.Thread(target=self.turnRight)
                    self.thread.start()
            elif key.char and key.char.lower() == 'a':
                if not self.action:
                    self.action = True
                    self.thread = threading.Thread(target=self.turnLeft)
                    self.thread.start()
            elif key.char and key.char.lower() == 'q':
                if not self.action:
                    self.action = True
                    self.thread = threading.Thread(target=self.MoveLeft)
                    self.thread.start()
            elif key.char and key.char.lower() == 'e':
                if not self.action:
                    self.action = True
                    self.thread = threading.Thread(target=self.MoveRight)
                    self.thread.start()
            elif key.char and key.char.lower() == 'h':
                if not self.action:
                    self.action = True
                    self.thread = threading.Thread(target=self.ShakeHand)
                    self.thread.start()
            elif key.char and key.char.lower() == 'j':
                if not self.action:
                    self.action = True
                    self.thread = threading.Thread(target=self.JumpForward)
                    self.thread.start()

            elif key.char and key.char.lower() == 'l':
                if not self.action:
                    self.action = True
                    self.thread = threading.Thread(target=self.Love)
                    self.thread.start()
            
        except AttributeError:
            if key == keyboard.Key.space:
                if self.standing:
                    self.client.StandDown()
                    time.sleep(1)
                    self.standing = False
                else:
                    self.client.StandUp()
                    time.sleep(self.dt)
                    self.standing = True
                    self.client.BalanceStand()
                    time.sleep(1)

    def on_release(self, key):
        try:
            if key.char and key.char.lower() in ['w', 's', 'd', 'a', 'q', 'e', 'h', 'j', 'l']:
                self.action = False  # Stop current action
                if self.thread.is_alive():
                    self.thread.join()  # Wait for movement thread to finish if still running
        except AttributeError:
            if key == keyboard.Key.space:
                pass  # Do nothing on space bar release

    def Start(self):
        with self.listener:
            self.listener.join()

# Robot state
robot_state = unitree_go_msg_dds__SportModeState_()
def HighStateHandler(msg: SportModeState_):
    global robot_state
    robot_state = msg

if __name__ == "__main__":
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:

        ChannelFactoryInitialize(0)
        
    sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub.Init(HighStateHandler, 10)
    time.sleep(1)

    test = TeleopKeyboard()
    test.GetInitState(robot_state)

    print("Teleop Start")

    test.Start()
