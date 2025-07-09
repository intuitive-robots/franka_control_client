"""
Defines message identifiers used in the communication protocol.
"""

from enum import IntEnum


class MsgID(IntEnum):
    """Message identifiers"""

    GET_STATE_REQ = 0x01
    GET_CONTROL_MODE_REQ = 0x02
    SET_CONTROL_MODE_REQ = 0x03
    GET_SUB_PORT_REQ = 0x04
    MOVE_TO_POSITION_REQ = 0x05

    GET_STATE_RESP = 0x51
    GET_CONTROL_MODE_RESP = 0x52
    SET_CONTROL_MODE_RESP = 0x53
    GET_SUB_PORT_RESP = 0x54

    ERROR = 0xFF
