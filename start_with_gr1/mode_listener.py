import time
import argparse
import json
import zenoh
from zenoh import Sample
import subprocess

# import logging

# logging.basicConfig(
#     level=logging.DEBUG,
#     format='%(asctime)s - %(levelname)s - %(message)s',
#     handlers=[
#         logging.FileHandler('./log/mode_listener.log'),
#         logging.StreamHandler()
#     ]
# )


class mode_listener():
# --- Command line argument parsing --- --- --- --- --- ---
    upper_control_mode = False
    def __init__(self):
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
                            # default = 'gr/mode/switch',
                            type=str,
                            help='The key expression matching queries to reply to.')
        parser.add_argument('--value', '-v', dest='value',
                            # default='/0',
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
        conf = zenoh.Config.from_file(args.config) if args.config is not None else zenoh.Config()
        if args.mode is not None:
            conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
        if args.connect is not None:
            conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
        if args.listen is not None:
            conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))
        # self.key = args.key
        # value = args.value
        self.complete = args.complete

        self.response_json = '0'
        self.value = "0"

        self.motor_list_file = '../../../RoCS/bin/MotorList/sources/motor_enable.json'
        self.on_file = '../../RoCS_motor_enable/ON/motor_enable.json'
        self.off_file = '../../RoCS_motor_enable/OFF/motor_enable.json'
        # self.grx_upper_file_path = '../'

        self.key_init = "gr/hand/init"
        self.key_stop = "gr/hand/stop"
        self.key_mode_switch = "gr/mode/switch"
        self.key_service_switch = "gr/service/switch"

        self.init_input = None
        self.mode_switch_input = None
        self.service_switch_input = None

        # self.logger = logging.getLogger(__name__)
        # self.logger.debug('mode_listener created with name: mode_listener')
        # self.logger.info('mode_listener init complete')   


    def replace_json_content(self, source_file_path, target_file_path):
        
        with open(source_file_path, 'r') as source_file:
            data = json.load(source_file)

        with open(target_file_path, 'w') as target_file:
            json.dump(data, target_file, indent=4)
        
        # self.logger.info('mode_listener replace complete')   


    # Zenoh code  --- --- --- --- --- --- --- --- --- --- ---

    def queryable_mode_switch_callback(self, query):

        print(f">> [Queryable ] Received Query '{query.selector}'" + (f" with value: {query.value.payload}" if query.value is not None else ""))

        parameter = query.selector.decode_parameters() # decode the value part of the selector as dict
        print(parameter)
        first_item = list(parameter.items())[0] #output the first item of parameter dict
        mode_switch_input = first_item[0] #output the first key, the parameter required
        # 'print(mode_switch_input)
        # prin't(type(mode_switch_input))

        if mode_switch_input == '0': 
            # self.logger.info('receive 0, mode_listener callback jason status')   
            json_file_path = self.motor_list_file
            try:
                with open(json_file_path, 'r') as file:
                    data = json.load(file)
            except (FileNotFoundError, json.JSONDecodeError) as e:
                print(f"Error reading JSON file: {e}")
                return
            self.response_json = json.dumps(data)
            query.reply(Sample(self.key_mode_switch, self.response_json))
            # self.logger.info('mode_listener callback jason status complete') 
            
        elif mode_switch_input == '2,0':
            # self.logger.info('receive 2.0, mode_listener callback off')  
            self.replace_json_content(self.off_file, self.motor_list_file)
            # result = subprocess.run(['gnome-terminal', '--', 'bash', '-c', './run_grx_upper.sh start; exec bash'])
            query.reply(Sample(self.key_mode_switch, self.value))
            # self.logger.info('mode_listener callback off complete')
        
        elif mode_switch_input == '2,1':
            # self.logger.info('receive 2.1, mode_listener callback on')  
            self.replace_json_content(self.on_file, self.motor_list_file)
            # result = subprocess.run(['gnome-terminal', '--', 'bash', '-c', './run_grx_upper.sh stop; exec bash'])
            query.reply(Sample(self.key_mode_switch, self.value))
            # self.logger.info('mode_listener callback on complete') 
        
    def queryable_init_callback(self, query):
        # self.logger.info('receive 0, mode_listener callback init')
        print(f">> [Queryable ] Received Query '{query.selector}'" + (f" with value: {query.value.payload}" if query.value is not None else ""))
        parameter = query.selector.decode_parameters() # decode the value part of the selector as dict
        
        first_item = list(parameter.items())[0] #output the first item of parameter dict
        self.init_input = first_item[0] #output the first key, the parameter required
        
        if self.upper_control_mode == False and self.init_input == '0':
            # result = subprocess.run(['gnome-terminal', '--', 'bash', '-c', '/home/gr1p24ap0058/remote_control_upper/APP_V5/start_with_gr1/run_grx_upper.sh start; exec bash'])
            # self.logger.info('receive 0, start server, mian file...')
            result = subprocess.Popen(["/home/gr1p24ap0058/remote_control_upper/APP_V5/start_with_gr1/run_grx_upper.sh", "start_algorithm"])
            # result.wait()
            # self.logger.info('receive 0, start server, mian file complete')
            # print("input", self.init_input)
            # print(val)
            # print(type(val))
            # self.upper_control_mode = True
            # self.Zeynoh_server_status = True
            query.reply(Sample(self.key_init, self.value))
            # self.logger.info('receive 0, current false, mode_listener callback 0')

        elif self.upper_control_mode == True:
            query.reply(Sample(self.key_init, "-1")) # tap init in RoCS mode
            # self.logger.info('receive 0, current true, mode_listener callback -1')

        # self.logger.info('receive 0, mode_listener callback init complete')

    
    def queryable_service_switch_callback(self, query):
        # self.logger.info('receive 0, service_switch callback')
        print(f">> [Queryable ] Received Query '{query.selector}'" + (f" with value: {query.value.payload}" if query.value is not None else ""))
        parameter = query.selector.decode_parameters() # decode the value part of the selector as dict
        
        first_item = list(parameter.items())[0] #output the first item of parameter dict
        self.service_switch_input = first_item[0] #output the first key, the parameter required
        
        if self.service_switch_input == '1':
            # result = subprocess.run(['gnome-terminal', '--', 'bash', '-c', '/home/gr1p24ap0058/remote_control_upper/APP_V5/start_with_gr1/run_grx_upper.sh start; exec bash'])
            # self.logger.info('receive 1, start server')
            result = subprocess.Popen(["/home/gr1p24ap0058/remote_control_upper/APP_V5/start_with_gr1/run_grx_upper.sh", "start_service"])
            # result.wait()
            # self.logger.info('receive 1, start server complete')
            
            query.reply(Sample(self.key_service_switch, self.value))
        
        elif self.service_switch_input == '0':
            # self.logger.info('receive 0, stop server')
            result = subprocess.Popen(["/home/gr1p24ap0058/remote_control_upper/APP_V5/start_with_gr1/run_grx_upper.sh", "stop_service"])
            # result.wait()
            # self.logger.info('receive 0, stop server complete')
            
            query.reply(Sample(self.key_service_switch, self.value))
        


    
    def queryable_stop_callback(self, query):
        # self.logger.info('receive 0, stop call back')
        print(f">> [Queryable ] Received Query '{query.selector}'" + (f" with value: {query.value.payload}" if query.value is not None else ""))
        parameter = query.selector.decode_parameters() # decode the value part of the selector as dict
        
        first_item = list(parameter.items())[0] #output the first item of parameter dict
        self.init_input = first_item[0] #output the first key, the parameter required
        # result = subprocess.Popen(["/home/gr1p24ap0058/remote_control_upper/APP_V5/start_with_gr1/run_grx_upper.sh", "stop"])
        # result.wait()
        # if self.upper_control_mode == True:
        self.upper_control_mode = False
        query.reply(Sample(self.key_stop, "0"))
        # self.logger.info('receive 0, upper_mde to 0, call back 0 complete')
        # elif self.upper_control_mode == False:
        #     query.reply(Sample(self.key_init, "-1")) # tap init in RoCS mode
            

    def quit_all(self):
        self.upper_control_mode = False

        # result = subprocess.run(['gnome-terminal', '--', 'bash', '-c', './run_grx_upper.sh stop; exec bash'])

    def main(self):

        # self.logger.info('mode_listener start up, wait for 10')
        time.sleep(10)
        # self.logger.info('start up complete')

        # initiate self.logger
        
        zenoh.init_logger()

        session = zenoh.open()
        # self.logger.info('zenoh init complete')

        queryable_init = session.declare_queryable(self.key_init, self.queryable_init_callback, self.complete)
        queryable_stop = session.declare_queryable(self.key_stop, self.queryable_stop_callback, self.complete)
        queryable_mode_switch = session.declare_queryable(self.key_mode_switch, self.queryable_mode_switch_callback, self.complete)
        queryable_service_switch = session.declare_queryable(self.key_service_switch, self.queryable_service_switch_callback, self.complete)
        
        print("waiting for init info")
        # self.logger.info('into loop, wait for input')
        while True:

            #print("upper_mode", self.upper_control_mode) 

            time.sleep(0.5)
            


if __name__ == "__main__":
    mode_switch = mode_listener()
    # mode_switch.replace_json_content(mode_switch.off_file, mode_switch.motor_list_file)
    mode_switch.main()


