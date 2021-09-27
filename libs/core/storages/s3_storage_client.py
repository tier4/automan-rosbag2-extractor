import os
import glob
import requests
import json
from core.storages import BaseStorageClient
from core.automan_client import AutomanClient
import pprint


class S3StorageClient(BaseStorageClient):

    def __init__(self, storage_config):
        super(S3StorageClient, self).__init__(storage_config)
        os.mkdir('/s3')
        self.rosbag_path = '/s3/rosbag.bag'
        self.extract_path = storage_config['output_dir']
        self.target_url = storage_config['target_url']
        self.storage_id = storage_config['storage_id']

    def download(self, url=None):
        if url is None:
            url = self.target_url
        req = requests.get(url, stream=True)
        if 200 <= req.status_code < 300:
            with open(self.rosbag_path, "wb") as f:
                for chunk in req.iter_content(chunk_size=1024):
                    f.write(chunk)
        else:
            print('status_code = ' + str(req.status_code))

    def upload(self, automan_info):
        jpg = glob.glob(self.extract_path+'*.jpg')
        pcd = glob.glob(self.extract_path+'*.pcd')
        for filepath in jpg + pcd:
            name = os.path.split(filepath)[1]
            data = {
                    'storage_id': str(self.storage_id),
                    'key': self.extract_path + name}
            res = AutomanClient.send_result(
                    automan_info, data, automan_info['presigned']).text
            presigned = json.loads(res)
            pprint.pprint(presigned)
            headers = {'content-type': 'application/octet-stream'}
            res = requests.put(
                    presigned['url'],
                    headers=headers,
                    data=open(filepath, 'rb')
                    )
            if res.status_code != 204:
                print('status_code=' + str(res.status_code) + ': ' + res.text)

    def list(self):
        pass

    def get_input_path(self):
        return self.rosbag_path

    def get_output_dir(self):
        return self.extract_path
