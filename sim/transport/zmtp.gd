extends RefCounted
## Minimal ZMTP 3 peer with REP semantics over one connected StreamPeerTCP.
##
## Supports the NULL security mechanism and a single REQ or DEALER peer, which covers a pyzmq
## REQ client. Each request is an envelope ending in an empty delimiter frame followed by exactly
## one body frame; replies repeat the envelope. PING commands are answered with PONG and other
## commands are ignored. Reads block; any failure leaves `error` set and the peer unusable.

const MAX_FRAME := 1 << 28
const PEER_TYPES := ["REQ", "DEALER"]
const FLAG_MORE := 0x01
const FLAG_LONG := 0x02
const FLAG_COMMAND := 0x04
var error := ""
var _peer: StreamPeerTCP

func handshake(peer: StreamPeerTCP) -> Error:
  _peer = peer
  var greeting := PackedByteArray()
  greeting.resize(64)
  greeting[0] = 0xff
  greeting[9] = 0x7f
  greeting[10] = 3
  greeting[11] = 0
  for index in range(4):
    greeting[12 + index] = "NULL".unicode_at(index)
  if not _write(greeting):
    return FAILED
  var remote := _read(64)
  if remote.is_empty():
    return FAILED
  var mechanism := remote.slice(12, 32)
  var null_mechanism := greeting.slice(12, 32)
  if remote[0] != 0xff or remote[9] != 0x7f or remote[10] < 3 or mechanism != null_mechanism:
    return _fail("Peer is not a ZMTP 3 NULL-mechanism endpoint")
  if not _write(_command("READY", _property("Socket-Type", "REP"))):
    return FAILED
  var ready := _read_frame()
  if ready.is_empty():
    return FAILED
  if not ready["command"] or _command_name(ready["body"]) != "READY":
    return _fail("Expected a READY command from the peer")
  var properties := _properties(ready["body"].slice(6))
  if not properties.get("socket-type", "") in PEER_TYPES:
    return _fail("Peer socket type must be one of %s" % [PEER_TYPES])
  return OK

## Returns {"envelope": Array[PackedByteArray], "body": PackedByteArray}, or {} on failure.
func receive() -> Dictionary:
  var frames: Array[PackedByteArray] = []
  while true:
    var frame := _read_frame()
    if frame.is_empty():
      return {}
    if frame["command"]:
      if _command_name(frame["body"]) == "PING" and not _pong(frame["body"]):
        return {}
      continue
    frames.append(frame["body"])
    if not frame["more"]:
      break
  var delimiter := -1
  for index in range(frames.size()):
    if frames[index].is_empty():
      delimiter = index
      break
  if delimiter < 0 or frames.size() != delimiter + 2:
    _fail("Request must be an envelope, an empty delimiter and one body frame")
    return {}
  return {"envelope": frames.slice(0, delimiter + 1), "body": frames[delimiter + 1]}

func send(envelope: Array[PackedByteArray], body: PackedByteArray) -> Error:
  var output := PackedByteArray()
  for frame in envelope:
    output.append_array(_frame(frame, FLAG_MORE))
  output.append_array(_frame(body, 0))
  return OK if _write(output) else FAILED

func _fail(message: String) -> Error:
  if error.is_empty():
    error = message
  return FAILED

func _write(bytes: PackedByteArray) -> bool:
  if _peer.put_data(bytes) != OK:
    _fail("Connection closed while writing")
    return false
  return true

func _read(count: int) -> PackedByteArray:
  if count == 0:
    return PackedByteArray()
  var result: Array = _peer.get_data(count)
  if result[0] != OK or result[1].size() != count:
    _fail("Connection closed while reading")
    return PackedByteArray()
  return result[1]

func _read_frame() -> Dictionary:
  var flags := _read(1)
  if flags.is_empty():
    return {}
  if flags[0] & ~(FLAG_MORE | FLAG_LONG | FLAG_COMMAND):
    _fail("Reserved ZMTP frame flags are set")
    return {}
  var size_bytes := _read(8 if flags[0] & FLAG_LONG else 1)
  if size_bytes.is_empty():
    return {}
  var size := 0
  for byte in size_bytes:
    size = (size << 8) | byte
  if size < 0 or size > MAX_FRAME:
    _fail("Frame exceeds %d bytes" % MAX_FRAME)
    return {}
  var body := _read(size)
  if size > 0 and body.is_empty():
    return {}
  return {"more": flags[0] & FLAG_MORE != 0, "command": flags[0] & FLAG_COMMAND != 0, "body": body}

func _frame(body: PackedByteArray, flags: int) -> PackedByteArray:
  var header := PackedByteArray()
  if body.size() > 0xff:
    header.append(flags | FLAG_LONG)
    for shift in range(56, -8, -8):
      header.append((body.size() >> shift) & 0xff)
  else:
    header.append(flags)
    header.append(body.size())
  header.append_array(body)
  return header

func _command(name: String, data: PackedByteArray) -> PackedByteArray:
  var body := PackedByteArray([name.length()])
  body.append_array(name.to_ascii_buffer())
  body.append_array(data)
  return _frame(body, FLAG_COMMAND)

func _command_name(body: PackedByteArray) -> String:
  if body.is_empty() or body.size() < 1 + body[0]:
    return ""
  return body.slice(1, 1 + body[0]).get_string_from_ascii()

func _property(name: String, value: String) -> PackedByteArray:
  var bytes := PackedByteArray([name.length()])
  bytes.append_array(name.to_ascii_buffer())
  var encoded := value.to_utf8_buffer()
  for shift in [24, 16, 8, 0]:
    bytes.append((encoded.size() >> shift) & 0xff)
  bytes.append_array(encoded)
  return bytes

## Parses READY metadata; property names are case-insensitive, so keys are lowercased.
func _properties(bytes: PackedByteArray) -> Dictionary:
  var result := {}
  var index := 0
  while index < bytes.size():
    var name_size := bytes[index]
    if index + 1 + name_size + 4 > bytes.size():
      break
    var name := bytes.slice(index + 1, index + 1 + name_size).get_string_from_ascii().to_lower()
    index += 1 + name_size
    var value_size := (bytes[index] << 24) | (bytes[index + 1] << 16) | (bytes[index + 2] << 8) | bytes[index + 3]
    index += 4
    if index + value_size > bytes.size():
      break
    result[name] = bytes.slice(index, index + value_size).get_string_from_utf8()
    index += value_size
  return result

## Answers PING (name, 2-byte TTL, context) with PONG carrying the same context.
func _pong(ping: PackedByteArray) -> bool:
  return _write(_command("PONG", ping.slice(7)))
