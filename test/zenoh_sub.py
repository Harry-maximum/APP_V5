import sys
import time
import argparse
import json
import zenoh
from zenoh import Reliability, Sample

# --- Command line argument parsing --- --- --- --- --- ---
parser = argparse.ArgumentParser(
    prog='z_sub',
    description='zenoh sub example')
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
                    default='gr/hand/rotation',
                    type=str,
                    help='The key expression to subscribe to.')
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
key = args.key

# Zenoh code  --- --- --- --- --- --- --- --- --- --- ---
def main():
    # initiate logging
    zenoh.init_logger()

    print("Opening session...")
    session = zenoh.open(conf)

    print("Declaring Subscriber on '{}'...".format(key))


    def listener(sample: Sample):
        print(f">> [Subscriber] Received {sample.kind} ('{sample.key_expr}': '{sample.payload.decode('utf-8')}')")
        print(type(sample.payload.decode('utf-8')))
        # a = sample.payload.decode('utf-8')

    # WARNING, you MUST store the return value in order for the subscription to work!!
    # This is because if you don't, the reference counter will reach 0 and the subscription
    # will be immediately undeclared.
    # while True:
    time.sleep(1)
    sub = session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

    print("Enter 'q' to quit...")
    c = '\0'
    while c != 'q':
        c = sys.stdin.read(1)
        if c == '':
            time.sleep(1)

    # Cleanup: note that even if you forget it, cleanup will happen automatically when 
    # the reference counter reaches 0
    # sub.undeclare()
    # session.close()
main()