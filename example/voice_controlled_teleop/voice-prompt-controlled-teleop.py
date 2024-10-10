import time
import sys
import threading
import queue
import sounddevice as sd
import json
from vosk import Model, KaldiRecognizer
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient

# Initialize the audio queue for voice commands
q = queue.Queue()

# Audio callback function
def callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

class TeleopRobot:
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
        self.lin_time = 3
        self.rot_time = 5

        # Motion and Rotation velocity
        self.lin_vel = 0.3
        self.rot_vel = 0.5

        # Create a sport client
        self.client = SportClient()  
        self.client.SetTimeout(10.0)  # 3.0
        self.client.Init()

        # Assume robot starts in standing position
        self.standing = False  
        
        # Start the command loop for voice commands
        self.voice_command_thread = threading.Thread(target=self.voice_command_loop)
        self.voice_command_thread.start()

    def voice_command_loop(self):
        model = Model('example/voice_controlled_teleop/large_model')
        rec = KaldiRecognizer(model, 16000)  # Use the correct sample rate
        with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype="int16", channels=1, callback=callback):
            while True:
                # print("Name the command: ")
                data = q.get()
                if rec.AcceptWaveform(data):
                    result = rec.Result()
                    result_rct = json.loads(result)  # Convert JSON string to Dict
                    prompt = result_rct['text'].strip()
                    print(prompt)  # print the input command

                    # Process the recognized voice command
                    self.process_command(prompt)

    def process_command(self, prompt):
        if prompt == 'тоқта':  # stop the program
            print("Stopping the teleop")
            quit()
        elif prompt == 'отыр': # sit down
            self.SitDown()
        elif prompt == 'тұр': # stand up
            self.StandUp()
        elif prompt == 'оңға':  # turn right
            self.TurnRight(rot_time=self.rot_time)
        elif prompt == 'солға':  # turn left
            self.TurnLeft(rot_time=self.rot_time)
        elif prompt == 'алға':  # Move forward
            self.MoveForward(lin_time=self.lin_time)
        elif prompt == 'артқа':  # Move backward
            self.MoveBackward(lin_time=self.lin_time)
        elif 'қол' in prompt:  # wave front-right leg (say hello)
            self.SayHello()
        else:
            print('[WARNING] Wrong Command:', prompt)
            
        

    def GetInitState(self, robot_state: SportModeState_):
        self.px0 = robot_state.position[0]
        self.py0 = robot_state.position[1]
        self.yaw0 = robot_state.imu_state.rpy[2]

    def SitDown(self):
        try:
            if self.standing:
                self.client.BalanceStand()
                time.sleep(1)
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
                time.sleep(self.dt)
                print("Robot is standing up")
            else:
                print("Robot is already standing")
        except AttributeError:
            print("Error: Attribute not found or not initialized properly.")

    def TurnRight(self, rot_time=5):
        try:
            if self.standing:
                time.sleep(self.small_dt)  # Some delay to ensure activation
                for i in range(0, rot_time):
                    self.client.Move(0.0, 0.0, -self.rot_vel)  # Turn CW (yaw rotation)
                    print('Turning right...')
                    time.sleep(self.dt)
                    i += 1
                print("Stopped turning right")
            else:
                print('Robot is in sitting position')

        except AttributeError:
            print("Error: Attribute not found or initialized properly")
    
    def TurnLeft(self, rot_time=5):
        try:
            if self.standing:
                time.sleep(self.small_dt)  # Some delay to ensure activation
                for i in range(0, rot_time):
                    self.client.Move(0.0, 0.0, self.rot_vel)
                    print('Turning left...')
                    time.sleep(self.dt)
                    i += 1
                print("Stopped turning left")
            else:
                print('Robot is in sitting position')

        except AttributeError:
            print("Error: Attribute not found or initialized properly")
    
    def MoveForward(self, lin_time=3):
        try:
            if self.standing:
                time.sleep(self.small_dt)  # Some delay to ensure activation
                for i in range(0, lin_time):
                    self.client.Move(self.lin_vel, 0.0, 0.0)
                    print('Moving forward...')
                    time.sleep(self.dt)
                    i += 1
                self.client.BalanceStand()
                time.sleep(self.small_dt)
                print("Stopped moving forward")
            else:
                print('Robot is in sitting position')

        except AttributeError:
            print("Error: Attribute not found or initialized properly")
    
    def MoveBackward(self, lin_time=3):
        try:
            if self.standing:
                time.sleep(self.small_dt)  # Some delay to ensure activation
                for i in range(0, lin_time):
                    self.client.Move(-self.lin_vel, 0.0, 0.0)
                    print('Moving backward...')
                    time.sleep(self.dt)
                    i += 1
                self.client.BalanceStand()
                time.sleep(self.small_dt)
                print("Stopped moving backward")
            else:
                print('Robot is in sitting position')

        except AttributeError:
            print("Error: Attribute not found or initialized properly")
    
    def SayHello(self):
        try:
            if self.standing:
                self.client.Hello()
                time.sleep(self.dt)
                self.client.BalanceStand()
                time.sleep(self.small_dt)
                print("Robot is saying Hello")
            else:
                print("Robot is in sitting position")
        except AttributeError:
            pass


    def Start(self):
        self.voice_command_thread.join()


## Main Program ##
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

    test = TeleopRobot()
    test.GetInitState(robot_state)

    print("Voice Teleop Start")
    test.Start()
