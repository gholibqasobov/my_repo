import time
import sys
import threading
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)




## Robot Control ##
class TeleopKeyboard:
    def __init__(self) -> None:
        # Time count
        self.t = 0
        self.small_dt = 0.1
        self.dt = 0.5

        # Initial position and yaw
        self.px0 = 0
        self.py0 = 0
        self.yaw0 = 0

        # Motion and Rotation time
        self.lin_time = 4
        self.rot_time = 5

        # Motion and Rotation velocity
        self.lin_vel = 0.3
        self.rot_vel = 0.5

        # Create a sport client
        self.client = SportClient()  
        self.client.SetTimeout(10.0)  # 10.0
        self.client.Init()

        self.standing = True  # Assume robot starts in standing position
        self.command_thread = threading.Thread(target=self.command_loop)
        self.command_thread.start()
    

    def command_loop(self):
        while True:
            prompt = input('Enter the command (sit down, stand up, stop, right): ').strip()
            if prompt == 'stop':
                print("Stopping the Teleop")
                break
            elif prompt == 'sit down':
                self.SitDown()
            elif prompt == 'stand up': 
                self.StandUp()
            elif prompt == 'right':
                self.TurnRight()
            else:
                print("Error: Wrong commmand. Exiting...")
                break
    
    def GetInitState(self, robot_state: SportModeState_):
        self.px0 = robot_state.position[0]
        self.py0 = robot_state.position[1]
        self.yaw0 = robot_state.imu_state.rpy[2]

    
    def SitDown(self):
        try:
            if self.standing:
                    self.client.StandDown()
                    time.sleep(self.dt)
                    self.standing = False
                    print("Robot is sitting down")
            else:
                print("Robot is already sitting")
                
        except AttributeError:
            print("Error: Attribute not found or not initialized properly.")


    def StandUp(self):
        try:
            if not self.standing:
                    self.client.StandUp()
                    time.sleep(self.dt)
                    self.standing = True
                    self.client.BalanceStand()
                    time.sleep(self.small_dt)
                    print("Robot is standing up")
            else:
                print("Robot is already standing")
        except AttributeError:
            print("Error: Attribute not found or not initialized properly.")
    
    def TurnRight(self):
        try:
            if self.standing:
                time.sleep(self.small_dt)  # Small delay to ensure activation
                for i in range(0, self.rot_time):
                    self.client.Move(0.0, 0.0, -self.rot_vel)  # Turn CW (yaw rotation)
                    print('Turning right...')
                    time.sleep(self.dt) 
                    i+=1
                
                self.client.Move(0.0, 0.0, 0.0)  # Stop turning
                print('Stopped turning right')
            else:
                print('The robot is in sitting state')
        except AttributeError:
            print("Error: Attribute not found or initialized properly.")
        except KeyboardInterrupt:
            self.client.Move(0.0, 0.0, 0.0)
            print("Interrupted and stopped turning right.")


    def Start(self):
        self.command_thread.join()






## Main Program

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
