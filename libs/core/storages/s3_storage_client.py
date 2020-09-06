import os
import glob
import requests
import json
from core.storages import BaseStorageClient
from core.automan_client import AutomanClient


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
        if req.status_code == 200:
            with open(self.rosbag_path, 'wb') as f:
                f.write(req.content)
        else:
            print('status_code = ' + str(req.status_code))

    def upload(self, automan_info):
        jpg = glob.glob(self.extract_path+'*.jpg')
        pcd = glob.glob(self.extract_path+'*.pcd')
        for file in jpg + pcd:
            name = os.path.split(file)[1]
            params = {
                    'storage_id': str(self.storage_id),
                    'key': self.extract_path + name}
            res = AutomanClient.send_get(
                    automan_info, automan_info['presigned'], params).text
            presigned = json.loads(res)
            with open(file, 'rb') as f:
                res = requests.post(
                        presigned['url'],
                        data=presigned['fields'],
                        files={'file': (file, f)}
                        )
                if res.status_code != 204:
                    print('status_code=' + str(res.status_code) + ': ' + res.text)

    def list(self):
        pass

    def get_input_path(self):
        return self.rosbag_path

    def get_output_dir(self):
        return self.extract_path
