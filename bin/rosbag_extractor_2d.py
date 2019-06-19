#!/usr/bin/env python
import argparse
import json
import traceback
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
from rosbag.bag import Bag
import sys
from pypcd import PointCloud
sys.path.append(os.path.join(os.path.dirname(__file__), '../libs'))
from core.storage_client_factory import StorageClientFactory
from core.automan_client import AutomanClient


class UnknownCalibrationFormatError(Exception):
    pass


class RosbagExtractor_2d(object):

    @classmethod
    def extract(cls, file_path, topics, output_dir, raw_data_info, calibfile=None):
        extrinsics_mat, camera_mat, dist_coeff = None, None, None
        if calibfile:
            try:
                calib_path = calibfile
                extrinsics_mat, camera_mat, dist_coeff = cls.__parse_calib(calib_path)
            except Exception:
                raise UnknownCalibrationFormatError
        candidates = raw_data_info['records']

        try:
            count = 0
            with Bag(file_path) as bag:
                # for topic, msg, t in bag.read_messages(topics=topics):
                for topic, msg, t in bag.read_messages():
                    if msg._type == 'sensor_msgs/Image':
                        count += 1
                        output_path = output_dir + str(candidates[topic]) \
                            + '_' + str(count).zfill(6)
                        cls.__process_image(msg, msg._type, output_path, camera_mat, dist_coeff)

            result = {
                'file_path': output_dir,
                'frame_count': count,
                'name': os.path.basename(path),  # FIXME
                'original_id': int(raw_data_info['original_id']),
                'candidates': raw_data_info['candidates'],
            }
            return result
        except Exception as e:
            # FIXME
            print(traceback.format_exc())
            raise(e)

    @staticmethod
    def __process_image(msg, type, file_path, camera_mat=None, dist_coeff=None):
        image = None
        if "compressed" in type:
            image = np.fromstring(msg.data, np.uint8)
            image = cv2.imdecode(image, cv2.IMREAD_COLOR).astype('f')
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

    storage_client = StorageClientFactory.create(
        args.storage_type,
        json.loads(args.storage_info)
    )
    storage_client.download()
    path = storage_client.get_input_path()
    output_dir = storage_client.get_output_dir()
    os.makedirs(output_dir)
    res = RosbagExtractor_2d.extract(path, [], output_dir, json.loads(args.raw_data_info))
    AutomanClient.send_result(json.loads(args.automan_info), res)
