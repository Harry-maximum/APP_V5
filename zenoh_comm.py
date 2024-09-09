
import time
import argparse
import json
import zenoh
from zenoh import  config, Sample, Value, Reliability
import threading

class zenoh_comm:
    def __init__(self, frequency):
        # --- Command line argument parsing --- --- --- --- --- ---
        parser = argparse.ArgumentParser(
            prog='z_comm',
            description='zenoh communication')
        parser.add_argument('--mode', '-m', dest='mode',
                            choices=['peer', 'client'],
                            type=str,
                            help='The zenoh session mode.')
        parser.add_argument('--connect', '-e', dest='connect',
                            metavar='ENDPOINT',
                            action='append',
                            type=str,
                            help='Endpoints to connect to.')
        parser.add_argument('--listen', '-l', dest='listen',
                            metavar='ENDPOINT',
                            action='append',
                            type=str,
                            help='Endpoints to listen on.')
        parser.add_argument('--key', '-k', dest='key',
                            # default = key,
                            type=str,
                            help='The key expression matching queries to reply to.')
        parser.add_argument('--value', '-v', dest='value',
                            default='0',
                            type=str,
                            help='The value to reply to queries.')
        parser.add_argument('--complete', dest='complete',
                            default=False,
                            action='store_true',
                            help='Declare the queryable as complete w.r.t. the key expression.')
        parser.add_argument('--config', '-c', dest='config',
                            metavar='FILE',
                            type=str,
                            help='A configuration file.')

        args = parser.parse_args()
        self.conf = zenoh.Config.from_file(args.config) if args.config is not None else zenoh.Config()
        if args.mode is not None:
            self.conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
        if args.connect is not None:
            self.conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
        if args.listen is not None:
            self.conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))
        # key = args.key
        self.value = args.value
        self.complete = args.complete
        # key = key
        self.frequency = frequency

        self.key_init = "gr/hand/init"
        self.key_stop = "gr/hand/stop"
        self.key_move = "gr/hand/move"
        self.key_rot = "gr/hand/rotation"
        self.key_grab = "gr/hand/grab"

        self.grab_complete = True
        self.init_complete = True
        self.stop_complete = True
    
        self.init_input = None
        self.stop_input = None
        self.move_input = None
        self.grab_input = None
        self.rot_input = None

        self.thread = threading.Thread(target = self.read_zenoh_input)
        self.thread.start()

# Zenoh code  --- --- --- --- --- --- --- --- --- --- ---


    def queryable_init_callback(self, query):

        

        print(f">> [Queryable ] Received Query '{query.selector}'" + (f" with value: {query.value.payload}" if query.value is not None else ""))

        parameter = query.selector.decode_parameters() # decode the value part of the selector as dict
        
        first_item = list(parameter.items())[0] #output the first item of parameter dict
        self.init_input = first_item[0] #output the first key, the parameter required
        
        # print("input", self.init_input)
        # print(val)
        # print(type(val))
    
        query.reply(Sample(self.key_init, self.value))
  
    def queryable_stop_callback(self, query):

        

        print(f">> [Queryable ] Received Query '{query.selector}'" + (f" with value: {query.value.payload}" if query.value is not None else ""))

        parameter = query.selector.decode_parameters() # decode the value part of the selector as dict
        
        first_item = list(parameter.items())[0] #output the first item of parameter dict
        self.stop_input = first_item[0] #output the first key, the parameter required
        
        self.stop_complete = False
        # print("stop_input", self.stop_input)
        
    
        query.reply(Sample(self.key_stop, self.value))

    def move_listener(self, sample: Sample):
        # print(f">> [Subscriber] Received {sample.kind} ('{sample.key_expr}': '{sample.payload.decode('utf-8')}')")
        self.move_input = sample.payload.decode('utf-8')
        # print(a)
        # print(type(a))
    
    def grab_listener(self, sample: Sample):
        # print(f">> [Subscriber] Received {sample.kind} ('{sample.key_expr}': '{sample.payload.decode('utf-8')}')")
        self.grab_input = sample.payload.decode('utf-8')
        self.grab_complete = False
        print("grab_input,:", self.grab_input)
        print(type(self.grab_input))
    
    def rot_listener(self, sample: Sample):
        # print(f">> [Subscriber] Received {sample.kind} ('{sample.key_expr}': '{sample.payload.decode('utf-8')}')")
        self.rot_input = sample.payload.decode('utf-8')
        # print(a)
        # print(type(a))

    def read_zenoh_input(self):
        # initiate logging
    
        
        zenoh.init_logger()

        session = zenoh.open(self.conf)


        # print("Declaring Queryable on '{}'...".format(key))

        # while cnt < (0.05 * self.frequency): # 0.05s of checking
        # # while True:   
        
        queryable_init = session.declare_queryable(self.key_init, self.queryable_init_callback, self.complete)
        queryable_stop = session.declare_queryable(self.key_stop, self.queryable_stop_callback, self.complete)
        subscriber_move = session.declare_subscriber(self.key_move, self.move_listener, reliability=Reliability.RELIABLE())
        subscriber_grab = session.declare_subscriber(self.key_grab, self.grab_listener, reliability=Reliability.RELIABLE())
        subscriber_rot = session.declare_subscriber(self.key_rot, self.rot_listener, reliability=Reliability.RELIABLE())
        
        # time.sleep(0.01)
        # print(self.input)
        # time.sleep(1/self.frequency)
        
        # queryable_init.undeclare()
        # session.close()

        while True:
            
            if self.grab_complete == True:
                self.grab_complete == False
                self.grab_input = None

            if self.init_complete == True:
                self.init_complete == False
                self.init_input = None

            if self.stop_complete == True:
                self.stop_complete == False
                self.stop_input = None

            # self.init_input = None
            # self.stop_input = None
            # self.grab_input = None
            self.move_input = None
            self.rot_input = None
            # print(self.move_input)
        
            # print("grab_input", self.grab_input)
            # print("stop_input", self.stop_input)

            time.sleep(0.01)


 