import abc

from grpc import Channel


class Client(abc.ABC):

    @abc.abstractmethod
    def __init__(self, channel: Channel) -> None:
        ...

    @abc.abstractmethod
    def Start(self) -> bool:
        ...

    @abc.abstractmethod
    def Stop(self) -> bool:
        ...
