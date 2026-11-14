import collections
import collections.abc
import inspect

for _attr in ("Mapping", "MutableMapping", "Sequence", "Iterable", "Callable", "MutableSet"):
    if not hasattr(collections, _attr) and hasattr(collections.abc, _attr):
        setattr(collections, _attr, getattr(collections.abc, _attr))
if not hasattr(inspect, "getargspec"):
    inspect.getargspec = inspect.getfullargspec


def _patch_msgpackrpc():
    """Ensure msgpack-rpc-python==0.4.1 works across Python 3.10-3.12, Tornado 4-6, and msgpack 0.5-1.x."""
    import msgpack
    import msgpackrpc.loop
    import msgpackrpc.transport.tcp as _tcp
    from msgpackrpc.error import RPCError, TransportError

    def _make_packer(encoding):
        try:
            return msgpack.Packer(encoding=encoding, default=lambda x: x.to_msgpack())
        except TypeError:
            return msgpack.Packer(use_bin_type=True, default=lambda x: x.to_msgpack())

    def _make_unpacker(encoding):
        try:
            return msgpack.Unpacker(encoding=encoding)
        except TypeError:
            return msgpack.Unpacker(raw=False, strict_map_key=False)

    class _DirectLoop:
        def __init__(self, loop=None):
            self._transport = None
            self._running = False

        @staticmethod
        def instance():
            return _DirectLoop()

        def start(self):
            if self._transport is not None:
                self._transport._read_step()

        def stop(self):
            self._running = False

        def attach_periodic_callback(self, callback, callback_time):
            pass

        def dettach_periodic_callback(self):
            pass

    class _DirectClientTransport:
        def __init__(self, session, address, reconnect_limit, encodings=("utf-8", None)):
            self._session = session
            self._address = address
            self._encodings = encodings
            self._reconnect_limit = reconnect_limit
            self._sock = None
            self._packer = _make_packer(encodings[0])
            self._unpacker = _make_unpacker(encodings[1])
            self._session._loop._transport = self

        def _ensure_connected(self):
            if self._sock is not None:
                return
            last_err = None
            for _ in range(max(1, int(self._reconnect_limit))):
                sock = None
                try:
                    sock = self._address.socket()
                    sock.setblocking(True)
                    if self._session._timeout:
                        sock.settimeout(float(self._session._timeout))
                    sock.connect(self._address.unpack())
                    self._sock = sock
                    return
                except Exception as exc:
                    last_err = exc
                    if sock is not None:
                        try:
                            sock.close()
                        except Exception:
                            pass
            self._session.on_connect_failed(TransportError(str(last_err or "Retry connection over the limit")))
            raise TransportError(str(last_err or "Retry connection over the limit"))

        def send_message(self, message, callback=None):
            self._ensure_connected()
            payload = self._packer.pack(message)
            self._sock.sendall(payload)
            if callback is not None:
                callback()

        def _read_step(self):
            self._ensure_connected()
            try:
                data = self._sock.recv(65536)
            except Exception as exc:
                self.close()
                self._session.on_connect_failed(TransportError(str(exc)))
                return
            if not data:
                self.close()
                self._session.on_connect_failed(TransportError("Connection closed by server"))
                return
            self._unpacker.feed(data)
            for message in self._unpacker:
                if len(message) not in (3, 4):
                    raise RPCError(f"Invalid MessagePack-RPC protocol: message = {message}")
                msgtype = message[0]
                if msgtype == msgpackrpc.message.RESPONSE:
                    self._session.on_response(message[1], message[2], message[3])

        def close(self):
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

    def _send_request(self, method, args):
        msgid = next(self._generator)
        future = msgpackrpc.session.Future(self._loop, self._timeout)
        self._request_table[msgid] = future
        self._transport.send_message([msgpackrpc.message.REQUEST, msgid, method, args])
        return future

    msgpackrpc.loop.Loop = _DirectLoop
    msgpackrpc.Loop = _DirectLoop
    msgpackrpc.session.Loop = _DirectLoop
    msgpackrpc.client.Loop = _DirectLoop
    msgpackrpc.session.Session.send_request = _send_request
    _tcp.ClientTransport = _DirectClientTransport


_patch_msgpackrpc()

from .client import *
from .utils import *
from .types import *

__version__ = "1.8.1"

