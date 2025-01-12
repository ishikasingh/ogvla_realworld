from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Empty(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class JointPositions(_message.Message):
    __slots__ = ("positions",)
    POSITIONS_FIELD_NUMBER: _ClassVar[int]
    positions: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, positions: _Optional[_Iterable[float]] = ...) -> None: ...

class Pose(_message.Message):
    __slots__ = ("pose",)
    POSE_FIELD_NUMBER: _ClassVar[int]
    pose: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, pose: _Optional[_Iterable[float]] = ...) -> None: ...

class GripperAction(_message.Message):
    __slots__ = ("value",)
    VALUE_FIELD_NUMBER: _ClassVar[int]
    value: float
    def __init__(self, value: _Optional[float] = ...) -> None: ...

class Result(_message.Message):
    __slots__ = ("ok", "err")
    OK_FIELD_NUMBER: _ClassVar[int]
    ERR_FIELD_NUMBER: _ClassVar[int]
    ok: Ok
    err: Err
    def __init__(self, ok: _Optional[_Union[Ok, _Mapping]] = ..., err: _Optional[_Union[Err, _Mapping]] = ...) -> None: ...

class Ok(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class Err(_message.Message):
    __slots__ = ("message",)
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    message: str
    def __init__(self, message: _Optional[str] = ...) -> None: ...
