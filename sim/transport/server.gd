extends SceneTree
## Serves one World to one ZeroMQ REQ client over TCP.
##
## Arguments after `--`: `--bind ADDRESS` (default 127.0.0.1), `--port N` (default 0, any free
## port), `--endpoint-file PATH` (receives tcp://ADDRESS:PORT once listening) and
## `--accept-timeout SECONDS` (default 60). Exits 0 after a close request and 1 when the
## connection fails or drops.

const Codec = preload("res://transport/msgpack.gd")
const Zmtp = preload("res://transport/zmtp.gd")
const Runtime = preload("res://world/runtime.gd")

func _initialize() -> void:
  call_deferred("_serve")

func _serve() -> void:
  var options := {"bind": "127.0.0.1", "port": "0", "endpoint-file": "", "accept-timeout": "60"}
  var args := OS.get_cmdline_user_args()
  for index in range(0, args.size(), 2):
    var key := args[index].trim_prefix("--")
    if not args[index].begins_with("--") or not options.has(key) or index + 1 >= args.size():
      _exit("Unknown or incomplete option: " + args[index])
      return
    options[key] = args[index + 1]
  var listener := TCPServer.new()
  if listener.listen(int(options["port"]), options["bind"]) != OK:
    _exit("Cannot listen on %s:%s" % [options["bind"], options["port"]])
    return
  var endpoint := "tcp://%s:%d" % [options["bind"], listener.get_local_port()]
  if not options["endpoint-file"].is_empty() and not _publish(endpoint, options["endpoint-file"]):
    _exit("Cannot write the endpoint file")
    return
  print("CADENCE_WORLD_LISTENING " + endpoint)
  var deadline := Time.get_ticks_msec() + int(float(options["accept-timeout"]) * 1000)
  while not listener.is_connection_available():
    if Time.get_ticks_msec() > deadline:
      _exit("No client connected before the accept timeout")
      return
    OS.delay_msec(2)
  var peer := listener.take_connection()
  listener.stop()
  peer.set_no_delay(true)
  var zmtp := Zmtp.new()
  if zmtp.handshake(peer) != OK:
    _exit("ZMTP handshake failed: " + zmtp.error)
    return
  var codec := Codec.new()
  var runtime := Runtime.new()
  while not runtime.closed:
    var message := zmtp.receive()
    if message.is_empty():
      runtime.close()
      _exit("Client connection ended: " + zmtp.error)
      return
    var request = codec.decode(message["body"])
    var reply: Dictionary
    if not codec.error.is_empty():
      reply = runtime.error_reply("malformed", codec.error)
    else:
      reply = await runtime.handle(request)
    var payload := codec.encode(reply)
    if not codec.error.is_empty():
      payload = codec.encode(runtime.error_reply("internal", "Cannot encode reply: " + codec.error))
    if zmtp.send(message["envelope"], payload) != OK:
      runtime.close()
      _exit("Cannot send reply: " + zmtp.error)
      return
  peer.disconnect_from_host()
  quit(0)

## Writes through a temporary file so readers never see a partial endpoint.
func _publish(endpoint: String, path: String) -> bool:
  var partial := path + ".partial"
  var file := FileAccess.open(partial, FileAccess.WRITE)
  if file == null:
    return false
  file.store_string(endpoint)
  file.close()
  return DirAccess.rename_absolute(partial, path) == OK

func _exit(message: String) -> void:
  printerr("CADENCE_WORLD_EXIT: " + message)
  quit(1)
