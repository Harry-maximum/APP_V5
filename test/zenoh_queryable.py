
# import sys
# import time
# import argparse
# import json
# import zenoh
# from zenoh import config, Sample, Value

# # --- Command line argument parsing --- --- --- --- --- ---
# parser = argparse.ArgumentParser(
#     prog='z_queryable',
#     description='zenoh queryable example')
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
# parser.add_argument('--key', '-k', dest='key',
#                     default='gr/hand/init',
#                     type=str,
#                     help='The key expression matching queries to reply to.')
# parser.add_argument('--value', '-v', dest='value',
#                     default='300',
#                     type=str,
#                     help='The value to reply to queries.')
# parser.add_argument('--complete', dest='complete',
#                     default=False,
#                     action='store_true',
#                     help='Declare the queryable as complete w.r.t. the key expression.')
# parser.add_argument('--config', '-c', dest='config',
#                     metavar='FILE',
#                     type=str,
#                     help='A configuration file.')

# args = parser.parse_args()
# conf = zenoh.Config.from_file(args.config) if args.config is not None else zenoh.Config()
# if args.mode is not None:
#     conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
# if args.connect is not None:
#     conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
# if args.listen is not None:
#     conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))
# key = args.key
# value = args.value
# complete = args.complete

# # input = ""

# # Zenoh code  --- --- --- --- --- --- --- --- --- --- ---


# def queryable_callback(query):
#     print(f">> [Queryable ] Received Query '{query.selector}'" + (f" with value: {query.value.payload}" if query.value is not None else ""))
  
#     val = query.value.payload.decode('utf-8')
#     # print(val)
#     # print(type(val))
#     # input = val
#     query.reply(Sample(key, value))

#     # if query.value is not None:
#     # # 尝试解码 payload
#     #     try:
#     #         payload = query.value.payload.decode('utf-8')
#     #         print(f">> [Queryable ] Received Query '{key}' with value: {payload}")
#     #     except UnicodeDecodeError:
#     #         print(f">> [Queryable ] Received Query '{key}' but failed to decode payload")
#     #         payload = "Error decoding payload"
#     # else:
#     #     print(f">> [Queryable ] Received Query '{key}' with no value")
#     #     payload = "No value received"
    
#     # 打印要返回的 value
#     # print(payload)
#     # print(type(payload))

    

# def main():
#     # initiate logging
#     zenoh.init_logger()

#     print("Opening session...")
#     session = zenoh.open(conf)

#     print("Declaring Queryable on '{}'...".format(key))

#     while True:
#         queryable = session.declare_queryable(key, queryable_callback, complete)
#         # print("huhuhuhuhhhhuhuhu")
#         # print(input)
#         # print(type(input))
        
#         # print(queryable)
#         # time.sleep(1)
#         # print("Press CTRL-C to quit...")
#     while True:
#         time.sleep(1)

#     queryable.undeclare()
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
from zenoh import config, Sample, Value

# --- Command line argument parsing --- --- --- --- --- ---
parser = argparse.ArgumentParser(
    prog='z_queryable',
    description='zenoh queryable example')
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
                    default=['tcp/0.0.0.0:8000'],
                    action='append',
                    type=str,
                    help='Endpoints to listen on.')
parser.add_argument('--key', '-k', dest='key',
                    default='gr/mode/switch',
                    type=str,
                    help='The key expression matching queries to reply to.')
parser.add_argument('--value', '-v', dest='value',
                    default='Queryable from Python!',
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


conf = zenoh.Config()
# {
#     zenoh.config.LISTEN: "tcp/0.0.0.0:8000"
# }
key = args.key
value = args.value
complete = args.complete

# Zenoh code  --- --- --- --- --- --- --- --- --- --- ---


def queryable_callback(query):
    print(f">> [Queryable ] Received Query '{query.selector}'" + (f" with value: {query.value.payload}" if query.value is not None else ""))
    query.reply(Sample(key, value))


def main():
    # initiate logging
    zenoh.init_logger()

    print("Opening session...")
    session = zenoh.open(conf)
    print("Declaring Queryable on '{}'...".format(key))

    # while True:
        # print("Declaring Queryable on '{}'...".format(key))
    queryable = session.declare_queryable(key, queryable_callback, complete)

    print("Press CTRL-C to quit...")
    
    print("Enter 'q' to quit...")
    c = '\0'
    while c != 'q':
        c = sys.stdin.read(1)
        if c == '':
            time.sleep(1)


    # queryable.undeclare()
    # session.close()

main()
