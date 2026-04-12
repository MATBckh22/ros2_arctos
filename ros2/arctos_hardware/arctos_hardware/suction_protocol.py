"""
CAN protocol constants for the Arctos suction gripper (CAN ID 7).

Frame format matches MKS convention:
  TX: [opcode, data..., CRC]
  CRC = (CAN_ID + sum(all bytes before CRC)) & 0xFF
"""

SUCTION_CAN_ID = 7

# PC -> Arduino opcodes
CMD_SET_SUCTION  = 0x01  # [pump_pwm, valve_state]
CMD_SET_PUMP_PWM = 0x02  # [pwm_value]
CMD_SET_VALVE    = 0x03  # [state]
CMD_QUERY_STATUS = 0xE0  # (no data)
CMD_SET_WATCHDOG = 0xE1  # [timeout_100ms]

# Arduino -> PC opcodes (in response byte 0)
RSP_SET_SUCTION  = 0x01
RSP_SET_PUMP     = 0x02
RSP_SET_VALVE    = 0x03
RSP_STATUS       = 0xE0
RSP_WATCHDOG     = 0xE1
RSP_FAULT_EVENT  = 0xFF

# Fault bits
FAULT_WATCHDOG    = 0x01
FAULT_OVERCURRENT = 0x02


def build_crc(can_id: int, data: list[int]) -> int:
    return (can_id + sum(data)) & 0xFF


def build_frame(can_id: int, opcode: int, data: list[int] | None = None) -> bytes:
    """Build a CAN frame payload with CRC appended."""
    payload = [opcode]
    if data:
        payload.extend(data)
    crc = build_crc(can_id, payload)
    payload.append(crc)
    return bytes(payload)


def validate_crc(can_id: int, frame: bytes) -> bool:
    """Validate CRC on a received frame (CRC is last byte)."""
    if len(frame) < 2:
        return False
    expected = build_crc(can_id, list(frame[:-1]))
    return frame[-1] == expected
