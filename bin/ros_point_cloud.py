from sensor_msgs.msg import PointField
import numpy as np

type_map = dict([
    (PointField.INT8, ('I', 1, np.dtype('int8'))),
    (PointField.UINT8, ('U', 1, np.dtype('uint8'))),
    (PointField.INT16, ('I', 2, np.dtype('int16'))),
    (PointField.UINT16, ('U', 2, np.dtype('uint16'))),
    (PointField.INT32, ('I', 4, np.dtype('int32'))),
    (PointField.UINT32, ('U', 4, np.dtype('uint32'))),
    (PointField.FLOAT32, ('F', 4, np.dtype('float32'))),
    (PointField.FLOAT64, ('F', 8, np.dtype('float64'))),
])
PREFIX = '__'
def pc_to_dtype(msg):
    offset = 0
    dtype_list = []
    for f in msg.fields:
        while offset < f.offset:
            dtype_list.append(('%s%d' % (PREFIX, offset), np.uint8))
            offset += 1
        _t, s, d = type_map[f.datatype]
        dtype_list.append((f.name, d))
        offset += s

    while offset < msg.point_step:
        dtype_list.append(('%s%d' % (PREFIX, offset), np.uint8))
        offset += 1

    return dtype_list

def pc_to_array(msg):
    dtype_list = pc_to_dtype(msg)

    arr = np.frombuffer(msg.data, dtype_list)

    arr = arr[[fname for fname, _type in dtype_list
        if not (fname[:len(PREFIX)] == PREFIX)]]

    return np.reshape(arr, (msg.height, msg.width))

def build_header(metadata):
    template = """\
VERSION {version}
FIELDS {fields}
SIZE {size}
TYPE {type}
COUNT {count}
WIDTH {width}
HEIGHT {height}
VIEWPOINT {viewpoint}
POINTS {points}
DATA {data}
"""

    md = metadata.copy()
    md['fields'] = ' '.join(metadata['fields'])
    md['size'] = ' '.join(map(str, metadata['size']))
    md['type'] = ' '.join(metadata['type'])
    md['count'] = ' '.join(map(str, metadata['count']))
    md['width'] = str(metadata['width'])
    md['height'] = str(metadata['height'])
    md['viewpoint'] = ' '.join(map(str, metadata['viewpoint']))
    md['points'] = str(metadata['points'])

    header = template.format(**md)
    return header

def build_fmtstr(metadata):
    fmtstr = []
    for t, c in zip(metadata['type'], metadata['count']):
        if t == 'F':
            fmtstr.extend(['%.10f'] * c)
        elif t == 'I':
            fmtstr.extend(['%d'] * c)
        elif t == 'U':
            fmtstr.extend(['%u'] * c)

    return fmtstr

def save_pc_msg(msg, fname):
    metadata = {
      'version': .7,
      'fields': None,
      'type': None,
      'size': None,
      'count': None,
      'width': msg.width,
      'height': msg.height,
      'viewpoint': [.0, .0, .0, 1.0, .0, .0, .0],
      'points': 0,
      'data': 'binary_compressed',
    }
    fields = []
    types = []
    sizes = []
    counts = []
    for field in msg.fields:
        fields.append(field.name)
        t, s, _d = type_map[field.datatype]
        types.append(t)
        sizes.append(s)
        counts.append(field.count)
    metadata['fields'] = fields
    metadata['type'] = types
    metadata['size'] = sizes
    metadata['count'] = counts

    arr = pc_to_array(msg)
    data = arr.reshape(-1)
    metadata['height'], metadata['width'] = arr.shape
    metadata['points'] = len(data)


    with open(fname, 'w') as f:
        compression = 'ascii'
        metadata['data'] = compression

        header = build_header(metadata)
        f.write(header)

        fmtstr = build_fmtstr(metadata)
        np.savetxt(f, data, fmt=fmtstr)

