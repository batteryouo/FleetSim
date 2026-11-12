extends RefCounted
## MessagePack codec for World protocol messages.
##
## Encodes null, bool, int, float (always float64), String, PackedByteArray (bin), Array and
## Dictionary. Decoding rejects extension types, duplicate map keys, unsigned values above the
## int64 range, truncated input, nesting deeper than MAX_DEPTH, and trailing bytes.
## After a failed call, `error` describes the problem.

const MAX_DEPTH := 32
var error := ""
var _buffer: StreamPeerBuffer

func encode(value: Variant) -> PackedByteArray:
  error = ""
  _buffer = StreamPeerBuffer.new()
  _buffer.big_endian = true
  _write_value(value, 0)
  return PackedByteArray() if not error.is_empty() else _buffer.data_array

func decode(bytes: PackedByteArray) -> Variant:
  error = ""
  _buffer = StreamPeerBuffer.new()
  _buffer.big_endian = true
  _buffer.data_array = bytes
  var value = _read_value(0)
  if error.is_empty() and _buffer.get_available_bytes() != 0:
    error = "Trailing bytes after the message"
  return null if not error.is_empty() else value

func _write_value(value: Variant, depth: int) -> void:
  if depth > MAX_DEPTH:
    error = "Nesting exceeds %d levels" % MAX_DEPTH
    return
  match typeof(value):
    TYPE_NIL:
      _buffer.put_u8(0xc0)
    TYPE_BOOL:
      _buffer.put_u8(0xc3 if value else 0xc2)
    TYPE_INT:
      _put_int(value)
    TYPE_FLOAT:
      _buffer.put_u8(0xcb)
      _buffer.put_double(value)
    TYPE_STRING, TYPE_STRING_NAME:
      var bytes := String(value).to_utf8_buffer()
      _header(bytes.size(), 0xa0, 31, 0xd9, 0xda, 0xdb)
      _buffer.put_data(bytes)
    TYPE_PACKED_BYTE_ARRAY:
      _header(value.size(), -1, -1, 0xc4, 0xc5, 0xc6)
      _buffer.put_data(value)
    TYPE_ARRAY:
      _header(value.size(), 0x90, 15, -1, 0xdc, 0xdd)
      for item in value:
        _write_value(item, depth + 1)
    TYPE_DICTIONARY:
      _header(value.size(), 0x80, 15, -1, 0xde, 0xdf)
      for key in value:
        _write_value(key, depth + 1)
        _write_value(value[key], depth + 1)
    _:
      error = "Cannot encode Variant type %s" % type_string(typeof(value))

func _put_int(value: int) -> void:
  if value >= 0:
    if value < 0x80:
      _buffer.put_u8(value)
    elif value <= 0xff:
      _buffer.put_u8(0xcc)
      _buffer.put_u8(value)
    elif value <= 0xffff:
      _buffer.put_u8(0xcd)
      _buffer.put_u16(value)
    elif value <= 0xffffffff:
      _buffer.put_u8(0xce)
      _buffer.put_u32(value)
    else:
      _buffer.put_u8(0xcf)
      _buffer.put_u64(value)
  elif value >= -32:
    _buffer.put_u8(value & 0xff)
  elif value >= -0x80:
    _buffer.put_u8(0xd0)
    _buffer.put_8(value)
  elif value >= -0x8000:
    _buffer.put_u8(0xd1)
    _buffer.put_16(value)
  elif value >= -0x80000000:
    _buffer.put_u8(0xd2)
    _buffer.put_32(value)
  else:
    _buffer.put_u8(0xd3)
    _buffer.put_64(value)

## Writes a length header; a negative tag marks a width the type does not have.
func _header(size: int, fixed: int, fixed_max: int, tag8: int, tag16: int, tag32: int) -> void:
  if fixed >= 0 and size <= fixed_max:
    _buffer.put_u8(fixed | size)
  elif tag8 >= 0 and size <= 0xff:
    _buffer.put_u8(tag8)
    _buffer.put_u8(size)
  elif size <= 0xffff:
    _buffer.put_u8(tag16)
    _buffer.put_u16(size)
  elif size <= 0xffffffff:
    _buffer.put_u8(tag32)
    _buffer.put_u32(size)
  else:
    error = "Length exceeds the MessagePack limit"

func _need(count: int) -> bool:
  if _buffer.get_available_bytes() < count:
    error = "Truncated message"
    return false
  return true

func _read_value(depth: int) -> Variant:
  if depth > MAX_DEPTH:
    error = "Nesting exceeds %d levels" % MAX_DEPTH
    return null
  if not _need(1):
    return null
  var tag := _buffer.get_u8()
  if tag <= 0x7f:
    return tag
  if tag >= 0xe0:
    return tag - 0x100
  if tag >= 0x80 and tag <= 0x8f:
    return _map(tag & 0x0f, depth)
  if tag >= 0x90 and tag <= 0x9f:
    return _array(tag & 0x0f, depth)
  if tag >= 0xa0 and tag <= 0xbf:
    return _string(tag & 0x1f)
  match tag:
    0xc0:
      return null
    0xc2:
      return false
    0xc3:
      return true
    0xc4, 0xc5, 0xc6:
      return _bytes(_length(tag - 0xc4))
    0xca:
      return _buffer.get_float() if _need(4) else null
    0xcb:
      return _buffer.get_double() if _need(8) else null
    0xcc:
      return _buffer.get_u8() if _need(1) else null
    0xcd:
      return _buffer.get_u16() if _need(2) else null
    0xce:
      return _buffer.get_u32() if _need(4) else null
    0xcf:
      if not _need(8):
        return null
      var value := _buffer.get_u64()
      if value < 0:
        error = "Unsigned integer exceeds the int64 range"
        return null
      return value
    0xd0:
      return _buffer.get_8() if _need(1) else null
    0xd1:
      return _buffer.get_16() if _need(2) else null
    0xd2:
      return _buffer.get_32() if _need(4) else null
    0xd3:
      return _buffer.get_64() if _need(8) else null
    0xd9, 0xda, 0xdb:
      return _string(_length(tag - 0xd9))
    0xdc, 0xdd:
      return _array(_length(tag - 0xdc + 1), depth)
    0xde, 0xdf:
      return _map(_length(tag - 0xde + 1), depth)
  error = "Unsupported MessagePack tag 0x%02x" % tag
  return null

## Reads a 1-, 2- or 4-byte length for width index 0, 1 or 2; -1 after an error.
func _length(width: int) -> int:
  if not _need([1, 2, 4][width]):
    return -1
  return [_buffer.get_u8, _buffer.get_u16, _buffer.get_u32][width].call()

func _bytes(size: int) -> Variant:
  if size < 0 or not _need(size):
    return null
  return _buffer.get_data(size)[1]

func _string(size: int) -> Variant:
  var bytes = _bytes(size)
  return null if bytes == null else bytes.get_string_from_utf8()

func _array(size: int, depth: int) -> Variant:
  # Every element takes at least one byte, so a larger count is necessarily truncated.
  if size < 0 or not _need(size):
    return null
  var result := []
  for index in range(size):
    var item = _read_value(depth + 1)
    if not error.is_empty():
      return null
    result.append(item)
  return result

func _map(size: int, depth: int) -> Variant:
  if size < 0 or not _need(2 * size):
    return null
  var result := {}
  for index in range(size):
    var key = _read_value(depth + 1)
    if not error.is_empty():
      return null
    if result.has(key):
      error = "Duplicate map key"
      return null
    var value = _read_value(depth + 1)
    if not error.is_empty():
      return null
    result[key] = value
  return result
