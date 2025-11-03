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


class RequestResult(IntEnum):
    SUCCESS = 0x00            # Operation completed successfully
    FAIL = 0x01               # Generic failure
    INVALID_ARG = 0x02        # Request payload invalid or out of range
    BUSY = 0x03               # Device is busy / command rejected temporarily
    UNSUPPORTED = 0x04        # Command not supported in current mode
    TIMEOUT = 0x05            # Operation timed out
    COMM_ERROR = 0x06         # Communication or CRC error
    INTERNAL_ERROR = 0x07     # Internal logic or hardware fault

