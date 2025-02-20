from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_idl


import csv
import argparse
import os


# adding command line arguments
p = argparse.ArgumentParser()
p.add_argument('-b', '--bag_path')
p.add_argument('-p', '--file_path')
p.add_argument('-i', '--idl_path')
p.add_argument('-t', '--topic')
args = p.parse_args()

# Read definitions to python strings.
idl_text = Path(args.idl_path).read_text()

# Plain dictionary to hold message definitions.
add_types = {}

# Add definitions from one idl file to the dict.
add_types.update(get_types_from_idl(idl_text))

bagpath = Path(args.bag_path)

typestore = get_typestore(Stores.ROS2_HUMBLE)
typestore.register(add_types)

# Create reader instance and open for reading.
with AnyReader([bagpath], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == args.topic]
    
    csv_file = open(args.file_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['x', 'y', 'z', 'pitch', 'yaw',
                         'waist', 'shoulder', 'elbow', 'wrist'])
    rows = []

    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        rows.append([msg.x, msg.y, msg.z, msg.pitch, msg.yaw, msg.waist, msg.shoulder, msg.elbow, msg.wrist])
        if len(rows) >= 1000:
            csv_writer.writerows(rows)
            rows = []




