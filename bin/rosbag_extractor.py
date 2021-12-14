#!/usr/bin/env python
import argparse
import json
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import rosbag2_py
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from ros_point_cloud import save_pc_msg
# import pprint


#from pypcd import PointCloud
#import pypcd
sys.path.append(os.path.join(os.path.dirname(__file__), '../libs'))
from core.storage_client_factory import StorageClientFactory
from core.automan_client import AutomanClient


class UnknownCalibrationFormatError(Exception):
    pass

class RosbagExtractor(object):

    @classmethod
    def extract(cls, automan_info, file_path, topics, output_dir, raw_data_info, calibfile=None):
        storage_options, converter_options = cls.__get_rosbag_options(file_path)

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        candidates, topics = cls.__get_candidates(
            automan_info, int(raw_data_info['project_id']), int(raw_data_info['original_id']), raw_data_info['records'])

        type_map = {}
        topic_msgs = {}
        for c in candidates:
            type_map[c['topic_name']] = c['msg_type']
            topic_msgs[c['topic_name']] = None

        try:
            frame_time = []
            count = 0
            while reader.has_next():
                (topic_name, data, t) = reader.read_next()
                if topic_name in topic_msgs:
                    msg_type = get_message(type_map[topic_name])
                    msg = deserialize_message(data, msg_type)
                    topic_msgs[topic_name] = msg
                if all(msg is not None for msg in topic_msgs.values()):
                    count += 1
                    timestamp = None
                    for c in candidates:
                        output_path = output_dir + str(c['candidate_id']) \
                            + '_' + str(count).zfill(6)
                        tgt_msg = topic_msgs[c['topic_name']]
                        topic_msgs[c['topic_name']] = None
                        if c['msg_type'] == 'sensor_msgs/msg/PointCloud2':
                            cls.__process_pcd(tgt_msg, output_path)
                        else:
                            cls.__process_image(tgt_msg, c['msg_type'], output_path)
                        timestamp = tgt_msg.header.stamp
                    frame_time.append({
                        'frame_number': count,
                        'secs': timestamp.sec,
                        'nsecs': timestamp.nanosec,
                    })

            name = os.path.basename(path)
            if 'name' in raw_data_info and len(raw_data_info['name']) > 0:
                name = raw_data_info['name']

            result = {
                'version': [2, 0, 0],
                'file_path': output_dir,
                'frame_count': count,
                'name': name,
                'original_id': int(raw_data_info['original_id']),
                'candidates': raw_data_info['candidates'],
                'frames': frame_time
            }
            # pprint.pprint(result)
            return result
        except Exception as e:
            print(e)
            raise(e)

    @staticmethod
    def __get_rosbag_options(path):
        storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        return storage_options, converter_options


    @staticmethod
    def __get_candidates(automan_info, project_id, original_id, selected_topics):
        path = '/projects/' + str(project_id) + '/originals/' + str(original_id) + '/candidates/'
        res = AutomanClient.send_get(automan_info, path).json()
        candidates = []
        topics = []
        for c in res["records"]:
            analyzed_info = json.loads(c['analyzed_info'])
            if analyzed_info['topic_name'] in selected_topics.keys():
                candidate = {
                    'candidate_id': c["candidate_id"],
                    'msg_type': analyzed_info['msg_type'],
                    'topic_name': analyzed_info['topic_name']
                }
                candidates.append(candidate)
                topics.append(analyzed_info['topic_name'])
        return candidates, topics

    @staticmethod
    def __process_pcd(msg, file_path):
        save_pc_msg(msg, file_path + '.pcd')

    @staticmethod
    def __process_image(msg, _type, file_path, camera_mat=None, dist_coeff=None):
        image = None
        if "Compressed" in _type:
            bridge = CvBridge()
            image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        else:
            bridge = CvBridge()
            image = bridge.imgmsg_to_cv2(msg, "bgr8").astype('f')

        if camera_mat and dist_coeff:
            image = cv2.undistort(image, camera_mat, dist_coeff, None, camera_mat)

        cv2.imwrite(file_path + ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

    @staticmethod
    def __parse_calib(calib_path):
        fs = cv2.FileStorage(calib_path, cv2.FILE_STORAGE_READ)
        camera_extrinsic_mat = fs.getNode("CameraExtrinsicMat").mat()
        camera_mat = fs.getNode("CameraMat").mat()
        dist_coeff = np.transpose(fs.getNode("DistCoeff").mat())
        return camera_extrinsic_mat, camera_mat, dist_coeff


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--storage_type', required=True)
    parser.add_argument('--storage_info', required=True)
    parser.add_argument('--automan_info', required=True)
    parser.add_argument('--raw_data_info', required=True)
    args = parser.parse_args()
    automan_info = json.loads(args.automan_info)
    print('automan_info: ' + args.automan_info)
    print('storage_info: ' + args.storage_info)

    storage_client = StorageClientFactory.create(
        args.storage_type,
        json.loads(args.storage_info)
    )
    storage_client.download()
    path = storage_client.get_input_path()
    output_dir = storage_client.get_output_dir()
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    res = RosbagExtractor.extract(
        automan_info, path, [], output_dir, json.loads(args.raw_data_info))
    if args.storage_type == 'AWS_S3':
        storage_client.upload(automan_info)
    res = AutomanClient.send_result(automan_info, res)
    print(res)

