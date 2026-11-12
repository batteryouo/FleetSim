"""Request-reply transports that carry encoded World protocol messages."""
import zmq

class ZmqTransport:
  """ZeroMQ REQ socket connected to one World endpoint.

  `request` sends one payload and blocks for its reply. A timeout raises TimeoutError and closes
  the socket, because a REQ socket cannot send again before it receives.
  """
  def __init__(self, endpoint: str, timeout: float = 60.0):
    self._context = zmq.Context.instance()
    self._socket = self._context.socket(zmq.REQ)
    self._socket.setsockopt(zmq.LINGER, 0)
    self._socket.setsockopt(zmq.RCVTIMEO, int(timeout * 1000))
    self._socket.setsockopt(zmq.SNDTIMEO, int(timeout * 1000))
    self._socket.connect(endpoint)
  def request(self, payload: bytes) -> bytes:
    if self._socket.closed:
      raise ConnectionError("The transport is closed")
    try:
      self._socket.send(payload)
      return self._socket.recv()
    except zmq.Again as error:
      self.close()
      raise TimeoutError("The World did not reply in time") from error
  def close(self) -> None:
    if not self._socket.closed:
      self._socket.close()
