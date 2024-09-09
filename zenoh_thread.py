import threading
import time
import zenoh
from zenoh_subscriber import zenoh_subscriber 
from zenoh_comm import zenoh_queryable


class zenoh_comm():
    def __init__(self, frequency):
        super().__init__()
        # self.key_init = "gr/hand/init"
        # self.key_stop = "gr/hand/stop"
        # self.key_grab = "gr/hand/grab"
        # self.key_move = "gr/hand/move"
        # self.key_rotation = "gr/hand/rotation"
        self.frequency = frequency

        self.init_input = None
        self.stop_input = None
        self.grab_input = None
        self.move_input = None
        self.rot_input = None

        # self.init_session = zenoh.open()
        # self.stop_session = zenoh.open()
        # self.grab_session = zenoh.open()
        # self.move_session = zenoh.open()
        # self.rot_session = zenoh.open()

        print("thread start!!")

        self.thread = threading.Thread(target = self.read_remote_input)
        self.thread.start()
        
    def read_remote_input(self):

        init_stop_queryable = zenoh_queryable(self.frequency)
        # stop_queryable = zenoh_queryable(self.frequency)
        # grab_move_rot_subscriber = zenoh_subscriber(self.frequency)
        # move_subscriber = zenoh_subscriber(self.frequency)
        # rot_subscriber = zenoh_subscriber(self.frequency)
        

        init_stop_queryable.Z_queryable()
        # print(self.init_input, self.stop_input)
        self.init_input = init_stop_queryable.init_input
        self.stop_input = init_stop_queryable.stop_input
        # self.grab_input, self.move_input, self.rot_input =  grab_move_rot_subscriber.Z_queryable()
        
        
        
        
        # self.stop_input = stop_queryable.input
        # grab_subscriber.Z_subscriber()
        # # self.grab_input = grab_subscriber.input
        # move_subscriber.Z_subscriber()
        # # self.move_input = move_subscriber.input
        # rot_subscriber.Z_subscriber()
        # self.rot_input = rot_subscriber.input

        # print("thread_init_input",self.init_input)
        # print("thread_stop_input",self.stop_input)
        # print("thread_grab_input",self.grab_input)
        # print("thread_move_input",self.move_input)
        # print("thread_rot_input",self.rot_input)
    

        # time.sleep(1/self.frequency)


