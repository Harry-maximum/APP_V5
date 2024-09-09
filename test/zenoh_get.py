# # import zenoh

# # import time

# # zenoh.init_logger()
# # session = zenoh.open()


# # key_init = "gr/hand/init"
# # key_stop = "gr/hand/stop"
# # key_move = "gr/hand/move"
# # key_rotation = "gr/hand/rotation"
# # key_grab = "gr/hand/grad"


# # def listener(Sample): # decode the zenoh information
# #     return Sample.payload

# # def handle_response(sample):
# #     # print(f"Received query on path: {path}")
    
# #     data = sample.payload.decode('utf-8')
# #     number = int(data)

# #     return number

# # def queryable_callback():
# #     # print(f">> [Queryable ] Received Query '{query.selector}'" + (f" with value: {query.value.payload}" if query.value is not None else ""))
# #     data = zenoh.Sample.payload
# #     # number = int(data)
    
# #     sample = zenoh.Sample(zenoh.Query.selector, "0")
# #     zenoh.Query.reply(sample)
# #     # return session.get(key)
    

# #     return data

# # def zenoh_input(self):

# #     # zenoh = Zenoh(Config())
# #     # session = zenoh.open()
# #     zenoh_input = {}
# #     move_input = self.session.declare_subscriber(self.key_move, self.listener)
# #     rotation_input = self.session.declare_subscriber(self.key_rotation, self.listener)
# #     grab_input = self.session.declare_subscriber(self.key_grab, self.listener)
# #     # init_input_queryable = self.session.declare_queryable(self.key_init, self.queryable_callback)
# #     init_input = self.session.get(self.key_init, self.queryable_callback)
# #     # init_input_queryable = self.session.declare_queryable(self.key_stop, self.queryable_callback)
# #     stop_input = self.session.get(self.key_stop, self.listener)

    
    
# #     zenoh_input = {
# #         "move_input": move_input,
# #         "rotation_input": rotation_input,
# #         "grab_input": grab_input,
# #         "init_input": init_input,
# #         "stop_input": stop_input,
# #     }  
    
# #     return zenoh_input

# # if __name__=="__main__":

# #     init_input = "No Input"

# #     # zenoh_input = {}
# #     # move_input = self.session.declare_subscriber(self.key_move, self.listener, reliability=Reliability.RELIABLE())
# #     # rotation_input = self.session.declare_subscriber(self.key_rotation, self.listener, reliability=Reliability.RELIABLE())
# #     # grab_input = self.session.declare_subscriber(self.key_grab, self.listener, reliability=Reliability.RELIABLE())
# #     # # init_input_queryable = self.session.declare_queryable(self.key_init, self.queryable_callback)
# #     # init_input = self.session.get(self.key_init, self.queryable_callback)
# #     # # init_input_queryable = self.session.declare_queryable(self.key_stop, self.queryable_callback)
# #     # stop_input = self.session.get(self.key_stop, self.listener)
# #     while True:

# #         time.sleep(1)
# #         init_input = session.get(key_init, lambda get_number: data = )
        
# #         # zenoh_input = {
# #         #     "move_input": move_input,
# #         #     "rotation_input": rotation_input, 
# #         #     "grab_input": grab_input,
# #         #     "init_input": init_input,
# #         #     "stop_input": stop_input,

# #         # }  

# #         print(init_input)

# #
# # Copyright (c) 2022 ZettaScale Technology
# #
# # This program and the accompanying materials are made available under the
# # terms of the Eclipse Public License 2.0 which is available at
# # http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# # which is available at https://www.apache.org/licenses/LICENSE-2.0.
# #
# # SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
# #
# # Contributors:
# #   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
# #

# import sys
# import time
# import argparse
# import json
# import zenoh
# from zenoh import config, QueryTarget

# # --- Command line argument parsing --- --- --- --- --- ---
# parser = argparse.ArgumentParser(
#     prog='z_get',
#     description='zenoh get example')
# parser.add_argument('--mode', '-m', dest='mode',
#                     choices=['peer', 'client'],
#                     type=str,
#                     help='The zenoh session mode.')
# parser.add_argument('--connect', '-e', dest='connect',
#                     metavar='ENDPOINT',
#                     action='append',
#                     type=str,
#                     help='Endpoints to connect to.')
# parser.add_argument('--listen', '-l', dest='listen',
#                     metavar='ENDPOINT',
#                     action='append',
#                     type=str,
#                     help='Endpoints to listen on.')
# parser.add_argument('--selector', '-s', dest='selector',
#                     default='demo/example/**',
#                     type=str,
#                     help='The selection of resources to query.')
# parser.add_argument('--target', '-t', dest='target',
#                     choices=['ALL', 'BEST_MATCHING', 'ALL_COMPLETE', 'NONE'],
#                     default='BEST_MATCHING',
#                     type=str,
#                     help='The target queryables of the query.')
# parser.add_argument('--value', '-v', dest='value',
#                     type=str,
#                     help='An optional value to send in the query.')
# parser.add_argument('--config', '-c', dest='config',
#                     metavar='FILE',
#                     type=str,
#                     help='A configuration file.')

# args = parser.parse_args()
# conf = zenoh.Config.from_file(
#     args.config) if args.config is not None else zenoh.Config()
# if args.mode is not None:
#     conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
# if args.connect is not None:
#     conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
# if args.listen is not None:
#     conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))
# # selector = args.selector
# selector = "gr/hand/init"
# target = {
#     'ALL': QueryTarget.ALL(),
#     'BEST_MATCHING': QueryTarget.BEST_MATCHING(),
#     'ALL_COMPLETE': QueryTarget.ALL_COMPLETE(),
# }.get(args.target)

# # Zenoh code  --- --- --- --- --- --- --- --- --- --- ---
# def main():
#     # initiate logging
#     zenoh.init_logger()

#     print("Opening session...")
#     session = zenoh.open(conf)

#     print("Sending Query '{}'...".format(selector))

#     while True:

#         replies = session.get(selector, zenoh.Queue(), target=target, value=args.value, consolidation=zenoh.QueryConsolidation.NONE())
#         for reply in replies.receiver:
#             try:
#                 print(">> Received ('{}': '{}')"
#                     .format(reply.ok.key_expr, reply.ok.payload.decode("utf-8")))
#             except:
#                 print(">> Received (ERROR: '{}')"
#                     .format(reply.err.payload.decode("utf-8")))


#     session.close()

# main()

#
# Copyright (c) 2022 ZettaScale Technology
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# which is available at https://www.apache.org/licenses/LICENSE-2.0.
#
# SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#
# Contributors:
#   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
#

import sys
import time
import argparse
import json
import zenoh
from zenoh import config, QueryTarget

# --- Command line argument parsing --- --- --- --- --- ---
parser = argparse.ArgumentParser(
    prog='z_get',
    description='zenoh get example')
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
parser.add_argument('--selector', '-s', dest='selector',
                    default='demo/example/**',
                    type=str,
                    help='The selection of resources to query.')
parser.add_argument('--target', '-t', dest='target',
                    choices=['ALL', 'BEST_MATCHING', 'ALL_COMPLETE', 'NONE'],
                    default='BEST_MATCHING',
                    type=str,
                    help='The target queryables of the query.')
parser.add_argument('--value', '-v', dest='value',
                    default='5000',
                    type=str,
                    help='An optional value to send in the query.')
parser.add_argument('--config', '-c', dest='config',
                    metavar='FILE',
                    type=str,
                    help='A configuration file.')

args = parser.parse_args()
conf = zenoh.Config.from_file(
    args.config) if args.config is not None else zenoh.Config()
if args.mode is not None:
    conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
if args.connect is not None:
    conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
if args.listen is not None:
    conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))
# selector = args.selector
selector = "gr/hand/init"
target = {
    'ALL': QueryTarget.ALL(),
    'BEST_MATCHING': QueryTarget.BEST_MATCHING(),
    'ALL_COMPLETE': QueryTarget.ALL_COMPLETE(),
}.get(args.target)

# Zenoh code  --- --- --- --- --- --- --- --- --- --- ---
def main():
    # initiate logging
    zenoh.init_logger()

    print("Opening session...")
    session = zenoh.open(conf)

    print("Sending Query '{}'...".format(selector))
    while True:
        time.sleep(3)
        replies = session.get(selector, zenoh.Queue(), target=target, value=args.value, consolidation=zenoh.QueryConsolidation.NONE())
        
        for reply in replies.receiver:
            try:
                print(">> Received ('{}': '{}')"
                    .format(reply.ok.key_expr, reply.ok.payload.decode("utf-8")))
            except:
                print(">> Received (ERROR: '{}')"
                    .format(reply.err.payload.decode("utf-8")))


    session.close()

main()