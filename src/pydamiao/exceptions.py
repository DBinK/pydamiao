class PydamiaoError(Exception):
    """Base exception for pydamiao."""


class SerialBusError(PydamiaoError):
    """Raised when the serial bus cannot send or receive safely."""


class SerialPortClosedError(SerialBusError):
    """Raised when an operation requires an open serial port."""


class ProtocolError(PydamiaoError):
    """Raised when the Damiao bridge protocol encounters malformed frames."""


class ReceiverThreadError(SerialBusError):
    """Raised when the background receiver thread stops unexpectedly."""
