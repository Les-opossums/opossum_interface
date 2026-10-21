# -*- coding: utf-8 -*-
"""GENERE AUTOMATIQUEMENT par opossum_interfaces/gen.py depuis protocol.yaml.
NE PAS MODIFIER A LA MAIN : editer protocol.yaml puis lancer `python3 gen.py`.

Exemple :
    from opossum_interfaces import eth_protocol as eth
    frame = eth.build_frame("CMD_GOAL_POSITION", eth.encode("CMD_GOAL_POSITION", x=1.0, y=0.5, t=0.0))
    hdr, name, payload = eth.parse_frame(frame)
    fields = eth.decode(name, payload)
"""

import struct

FRAME_MAGIC = 0xC0DE
PROTOCOL_VERSION = 1
PROTOCOL_HASH = 0x50BCB662
MAX_PAYLOAD = 512
MSG_RAW_TEXT = 0xFF
FRAME_CRC_OFFSET = 12

# nom -> (port, direction vue du Zynq, framed)
CHANNELS = {
    'DEBUG': (5000, 'tx', True),
    'TELEMETRY': (5001, 'tx', True),
    'CMD': (5002, 'rx', True),
    'RAW_CMD': (5003, 'rx', False),
}

# nom -> (id, canal, kind) ; kind : None | 'text' | 'struct'
MESSAGES = {
    'HEARTBEAT': (0x01, 'TELEMETRY', 'struct'),
    'DEBUG_TEXT': (0x02, 'DEBUG', 'text'),
    'ODOM': (0x10, 'TELEMETRY', 'struct'),
    'IMU': (0x11, 'TELEMETRY', 'struct'),
    'MOTOR_STATE': (0x12, 'TELEMETRY', None),
    'ROBOT_STATE': (0x13, 'TELEMETRY', 'struct'),
    'AU': (0x14, 'TELEMETRY', 'struct'),
    'LEASH': (0x15, 'TELEMETRY', 'struct'),
    'LIDAR_SCAN_CHUNK': (0x16, 'TELEMETRY', 'struct'),
    'CMD_GENERIC': (0x20, 'CMD', 'text'),
    'CMD_GOAL_POSITION': (0x21, 'CMD', 'struct'),
    'CMD_SET_LIDAR': (0x22, 'CMD', 'struct'),
    'CMD_SET_CAMERA_1': (0x23, 'CMD', 'struct'),
    'CMD_SET_CAMERA_2': (0x24, 'CMD', 'struct'),
    'CMD_SET_CAMERA_3': (0x25, 'CMD', 'struct'),
    'CMD_SET_SPEED_MAX': (0x26, 'CMD', None),
    'CMD_BLOCK': (0x27, 'CMD', 'struct'),
    'CMD_FREE': (0x28, 'CMD', 'struct'),
}
MESSAGE_BY_ID = {v[0]: k for k, v in MESSAGES.items()}

_HEADER = struct.Struct("<HBBHIHH")
HEADER_SIZE = _HEADER.size
HEADER_FIELDS = ['magic', 'version', 'msg_type', 'seq', 'timestamp_us', 'payload_len', 'crc16']

# nom -> (format fixe, noms aplatis, tableau variable (nom, champ compteur, format, noms) ou None)
PAYLOADS = {
    'HEARTBEAT': ('<IBBI', ['uptime_ms', 'state', 'flags', 'protocol_hash'], None),
    'ODOM': ('<ffffff', ['x', 'y', 'theta', 'vx', 'vy', 'omega'], None),
    'IMU': ('<ffffffffff', ['qw', 'qx', 'qy', 'qz', 'gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z'], None),
    'ROBOT_STATE': ('<IffffffB', ['timestamp_ms', 'x', 'y', 'theta', 'speed_linear', 'speed_direction', 'speed_angular', 'motion_done'], None),
    'AU': ('<BI', ['state', 'timestamp_ms'], None),
    'LEASH': ('<BI', ['state', 'timestamp_ms'], None),
    'LIDAR_SCAN_CHUNK': ('<BIIHHH', ['lidar_id', 'scan_id', 'timestamp_ms', 'chunk_index', 'chunk_count', 'point_count'], ('points', 'point_count', 'HH', ['distance_mm', 'angle_cdeg'])),
    'CMD_GOAL_POSITION': ('<fff', ['x', 'y', 't'], None),
    'CMD_SET_LIDAR': ('<fffI', ['x', 'y', 't', 'delay'], None),
    'CMD_SET_CAMERA_1': ('<fffIfff', ['x', 'y', 't', 'delay', 'noise_x', 'noise_y', 'noise_t'], None),
    'CMD_SET_CAMERA_2': ('<fffIfff', ['x', 'y', 't', 'delay', 'noise_x', 'noise_y', 'noise_t'], None),
    'CMD_SET_CAMERA_3': ('<fffIfff', ['x', 'y', 't', 'delay', 'noise_x', 'noise_y', 'noise_t'], None),
    'CMD_BLOCK': ('<B', ['value'], None),
    'CMD_FREE': ('<B', ['value'], None),
}


def crc16_ccitt(data, crc=0xFFFF):
    """CRC16-CCITT (poly 0x1021, init 0xFFFF) sur l'en-tete (crc16 = 0) puis le payload"""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _nest(flat):
    """{"a.b": 1, "p[0].x": 2} -> {"a": {"b": 1}, "p": [{"x": 2}]}"""
    out = {}
    for key, val in flat.items():
        cur = out
        parts = key.split('.')
        for i, part in enumerate(parts):
            last = i == len(parts) - 1
            if '[' in part:
                name, idx = part[:-1].split('[')
                lst = cur.setdefault(name, [])
                idx = int(idx)
                while len(lst) <= idx:
                    lst.append({})
                if last:
                    lst[idx] = val
                else:
                    cur = lst[idx]
            elif last:
                cur[part] = val
            else:
                cur = cur.setdefault(part, {})
    return out


def _flatten(obj, prefix=''):
    out = {}
    if isinstance(obj, dict):
        for k, v in obj.items():
            out.update(_flatten(v, f'{prefix}.{k}' if prefix else k))
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            out.update(_flatten(v, f'{prefix}[{i}]'))
    else:
        out[prefix] = obj
    return out


def encode(name, **fields):
    """Encode le payload d'un message structure. Les tableaux variables se passent en liste."""
    fmt, names, var = PAYLOADS[name]
    items = []
    if var:
        items = list(fields.pop(var[0], []))
        fields[var[1]] = len(items)
    flat = _flatten(fields)
    missing = [n for n in names if n not in flat]
    if missing:
        raise KeyError(f'{name} : champs manquants {missing}')
    data = struct.pack(fmt, *(flat[n] for n in names))
    if var:
        vs = struct.Struct('<' + var[2])
        for it in items:
            f = _flatten(it) if isinstance(it, dict) else {'': it}
            data += vs.pack(*(f[n] for n in var[3]))
    return data


def decode(name, payload):
    """Decode un payload ; retourne un dict (texte : str, format non defini : bytes)."""
    kind = MESSAGES[name][2]
    if kind == 'text':
        return payload.decode('ascii', errors='replace')
    if kind is None:
        return bytes(payload)
    fmt, names, var = PAYLOADS[name]
    size = struct.calcsize(fmt)
    if len(payload) < size:
        raise ValueError(f'{name} : payload trop court ({len(payload)} < {size})')
    out = _nest(dict(zip(names, struct.unpack_from(fmt, payload))))
    if var:
        vs = struct.Struct('<' + var[2])
        n = out[var[1]]
        if len(payload) != size + n * vs.size:
            raise ValueError(f'{name} : taille {len(payload)} incoherente avec {var[1]}={n}')
        items = []
        for i in range(n):
            vals = vs.unpack_from(payload, size + i * vs.size)
            items.append(vals[0] if var[3] == [''] else _nest(dict(zip(var[3], vals))))
        out[var[0]] = items
    elif len(payload) != size:
        raise ValueError(f'{name} : taille {len(payload)} != {size}')
    return out


def build_frame(name, payload=b'', seq=0, timestamp_us=0):
    """Construit une trame complete (en-tete + CRC + payload)."""
    if isinstance(payload, str):
        payload = payload.encode('ascii')
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload trop grand')
    hdr = dict(magic=FRAME_MAGIC, version=PROTOCOL_VERSION, msg_type=MESSAGES[name][0],
               seq=seq & 0xFFFF, timestamp_us=timestamp_us & 0xFFFFFFFF,
               payload_len=len(payload), crc16=0)
    raw = bytearray(_HEADER.pack(*(hdr[n] for n in HEADER_FIELDS)) + payload)
    struct.pack_into('<H', raw, FRAME_CRC_OFFSET, crc16_ccitt(raw))
    return bytes(raw)


def parse_frame(frame):
    """Verifie et decoupe une trame ; retourne (en-tete dict, nom du message, payload)."""
    if len(frame) < HEADER_SIZE:
        raise ValueError('trame trop courte')
    hdr = dict(zip(HEADER_FIELDS, _HEADER.unpack_from(frame)))
    if hdr['magic'] != FRAME_MAGIC:
        raise ValueError('magic invalide')
    if HEADER_SIZE + hdr['payload_len'] != len(frame):
        raise ValueError('payload_len incoherent')
    raw = bytearray(frame)
    struct.pack_into('<H', raw, FRAME_CRC_OFFSET, 0)
    if crc16_ccitt(raw) != hdr['crc16']:
        raise ValueError('CRC invalide')
    name = MESSAGE_BY_ID.get(hdr['msg_type'], 'UNKNOWN')
    return hdr, name, bytes(frame[HEADER_SIZE:])
