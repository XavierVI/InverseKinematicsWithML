from pathlib import Path

import csv

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_idl, get_types_from_msg

# Read definitions to python strings.
idl_text = Path('../MARIAM/install/arm_controller/share/arm_controller/msg/PositionAndJoints.idl').read_text()

# Plain dictionary to hold message definitions.
add_types = {}

# Add definitions from one idl file to the dict.
add_types.update(get_types_from_idl(idl_text))

bagpath = Path('../datasets/joint_positions_data_1000')

typestore = get_typestore(Stores.ROS2_HUMBLE)
typestore.register(add_types)

# Create reader instance and open for reading.
with AnyReader([bagpath], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == '/position_data']

    csv_file = open('../datasets/joint_positions_data_1000.csv', 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['x', 'y', 'pitch', 'shoulder', 'elbow', 'wrist'])
    rows = []

    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        rows.append([msg.x, msg.y, msg.pitch, msg.shoulder, msg.elbow, msg.wrist])
        if len(rows) >= 1000:
            csv_writer.writerows(rows)
            rows = []
    # print(connections[0].msgtype)





