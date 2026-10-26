"""7 bytes: AA 55 sequence command shape CRC16-CCITT-FALSE (little endian).

Commands: 0 stop/lost, 1 forward, 2 left, 3 right.
Shape: 0 none, 1 circle, 2 star, 3 square, 4 diamond, 5 plus, 6 triangle.
The legacy packet has no angle. Additional AA56 packets carry unified control.
"""
import struct


def crc16(data):
    crc = 0xffff
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xffff if crc & 0x8000 else (crc << 1) & 0xffff
    return crc


def encode(result, sequence, shape_id=0):
    if shape_id not in (0, 1, 2, 3, 4, 5, 6):
        raise ValueError("shape_id must be 0..6")
    commands = {"LOST": 0, "STRAIGHT": 1, "LEFT": 2, "RIGHT": 3}
    command = commands[result["state"]] if result["valid"] and not result["stop"] else 0
    body = bytes((0xaa, 0x55, sequence & 255, command, shape_id))
    return body + struct.pack("<H", crc16(body))


def encode_angle(result, sequence):
    """8 bytes AA 56 seq status int16_deci_degrees crc16, little endian.
    status 0=invalid, 1=no adjustment, 2=generic angle,
    3=left turn/left correction with optional signed angle,
    4=right correction, 5=wait for board.
    For status 3: negative angle increases left turn, positive reduces it;
    zero asks the STM32 to use its default 2090 target.
    """
    status = result.get('status', 0 if not result['valid'] else 2 if result['active'] else 1)
    value = int(round(result['angle']*10)) if status in (2,3,4) else 0
    value = max(-900, min(900, value))
    body = bytes((0xaa,0x56,sequence & 255,status))+struct.pack('<h',value)
    return body+struct.pack('<H',crc16(body))


def encode_obstacle(result, sequence):
    """8 bytes AA 58 seq flags int16 bar-angle-in-0.1deg CRC16."""
    present=1 if result.get('present') else 0
    value=int(round(result.get('angle',0.0)*10)) if present else 0
    value=max(-450,min(450,value))
    body=bytes((0xaa,0x58,sequence & 255,present))+struct.pack('<h',value)
    return body+struct.pack('<H',crc16(body))

