#!/usr/bin/env python3
"""Push built disting NT plugin .o files to a connected module over MIDI SysEx.

Reimplements the wake / mkdir / chunked-upload / rescan SysEx handshake that
nt_helper (https://github.com/thorinside/nt_helper) uses to install plugins,
so a plugin can be deployed without ejecting the SD card.

Requires: pip install mido python-rtmidi
"""

import argparse
import os
import sys
import time

try:
    import mido
except ImportError:
    sys.exit(
        "nt_push.py requires the 'mido' and 'python-rtmidi' packages.\n"
        "Install them with: pip install mido python-rtmidi"
    )

SYSEX_START = 0xF0
SYSEX_END = 0xF7
MANUFACTURER_ID = (0x00, 0x21, 0x27)
NT_PREFIX = 0x6D

OP_WAKE = 0x07
OP_SDCARD = 0x7A

SD_OP_LISTING = 1
SD_OP_UPLOAD = 4
SD_OP_RENAME = 5
SD_OP_NEW_FOLDER = 7
SD_OP_RESCAN = 8

CHUNK_SIZE = 512
CHUNK_DELAY = 0.05
REPLY_TIMEOUT = 0.3


def checksum(payload):
    return (-sum(payload)) & 0x7F


def build_header(sysex_id):
    return [SYSEX_START, *MANUFACTURER_ID, NT_PREFIX, sysex_id & 0x7F]


def encode_ascii_path(path):
    out = []
    for ch in path:
        code = ord(ch)
        if code == 0 or code > 0x7F:
            raise ValueError(f"path must be 7-bit ASCII: {path!r}")
        out.append(code)
    return out


def bytes_to_nibbles(data):
    nibbles = []
    for b in data:
        nibbles.append((b >> 4) & 0x0F)
        nibbles.append(b & 0x0F)
    return nibbles


def encode_position_or_count(value):
    return [
        0, 0, 0, 0, 0,
        (value >> 28) & 0x0F,
        (value >> 21) & 0x7F,
        (value >> 14) & 0x7F,
        (value >> 7) & 0x7F,
        value & 0x7F,
    ]


def sdcard_message(sysex_id, payload):
    # Checksum covers only `payload` (the opcode byte onward), not the
    # leading 0x7A sd-card-operation message-type byte.
    return build_header(sysex_id) + [OP_SDCARD, *payload] + [checksum(payload)] + [SYSEX_END]


def wake_message(sysex_id):
    # No checksum on the wake message.
    return build_header(sysex_id) + [OP_WAKE] + [SYSEX_END]


def directory_create_message(sysex_id, path):
    return sdcard_message(sysex_id, [SD_OP_NEW_FOLDER, *encode_ascii_path(path)])


def rescan_message(sysex_id):
    return sdcard_message(sysex_id, [SD_OP_RESCAN])


def upload_chunk_message(sysex_id, path, position, data, create_always):
    payload = [
        SD_OP_UPLOAD,
        *encode_ascii_path(path),
        0,  # null terminator
        1 if create_always else 0,
        *encode_position_or_count(position),
        *encode_position_or_count(len(data)),
        *bytes_to_nibbles(data),
    ]
    return sdcard_message(sysex_id, payload)


def find_port(names, requested, kind):
    if requested:
        matches = [n for n in names if requested.lower() in n.lower()]
        if not matches:
            sys.exit(f"No MIDI {kind} port matching {requested!r}. Available: {names}")
        if len(matches) > 1 and requested not in names:
            sys.exit(f"Ambiguous MIDI {kind} port {requested!r}, matches: {matches}")
        return matches[0] if requested not in names else requested
    matches = [n for n in names if "disting" in n.lower()]
    if len(matches) == 1:
        return matches[0]
    if not names:
        sys.exit(f"No MIDI {kind} ports found.")
    sys.exit(
        f"Could not auto-select a MIDI {kind} port (candidates: {matches or names}).\n"
        f"Pass --port \"<name>\" to pick one explicitly."
    )


def describe_reply(data):
    """Best-effort human-readable hint: replies are
    [mfr(3), prefix, sysexid, 0x7A, status, ...payload], where the
    payload is sometimes an ASCII status message."""
    data = bytes(data)
    status = data[6] if len(data) > 6 else None
    payload = bytes(b for b in data[7:] if 0x20 <= b < 0x7F).decode("ascii", "ignore")
    prefix = f"status={status:#04x} " if status is not None else ""
    return f"{prefix}{payload!r}" if payload else f"{prefix}{data.hex()}"


def drain_replies(inport, timeout=REPLY_TIMEOUT):
    """Best-effort: print any SysEx replies seen within `timeout` seconds."""
    deadline = time.time() + timeout
    seen = []
    while time.time() < deadline:
        for msg in inport.iter_pending():
            if msg.type == "sysex":
                seen.append(describe_reply(msg.data))
        if seen:
            break
        time.sleep(0.01)
    if seen:
        print(f"    device reply: {' | '.join(seen)}")


def send(outport, inport, message, note=None):
    outport.send(mido.Message("sysex", data=message[1:-1]))
    if note:
        print(f"  {note}")
    if inport is not None:
        drain_replies(inport)


def ensure_directories(outport, inport, sysex_id, dest_dir):
    parts = [p for p in dest_dir.split("/") if p]
    path = ""
    for part in parts:
        path += "/" + part
        send(outport, inport, directory_create_message(sysex_id, path))


def upload_file(outport, inport, sysex_id, local_path, remote_path):
    with open(local_path, "rb") as f:
        data = f.read()

    print(f"Uploading {local_path} -> {remote_path} ({len(data)} bytes)")
    position = 0
    while position < len(data) or (position == 0 and len(data) == 0):
        chunk = data[position:position + CHUNK_SIZE]
        send(
            outport,
            inport,
            upload_chunk_message(sysex_id, remote_path, position, chunk, position == 0),
        )
        position += len(chunk)
        print(f"  {position}/{len(data)} bytes")
        if position < len(data):
            time.sleep(CHUNK_DELAY)
        if len(chunk) == 0:
            break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="*", help="Built .o plugin file(s) to install")
    parser.add_argument("--port", default=os.environ.get("MIDI_PORT"),
                         help="MIDI port name (or substring). Default: auto-detect a port containing 'disting'")
    parser.add_argument("--sysex-id", type=int, default=int(os.environ.get("NT_SYSEX_ID", "0")),
                         help="disting NT SysEx ID (default: 0)")
    parser.add_argument("--dest-dir", default="/programs/plug-ins",
                         help="Target directory on the SD card (default: /programs/plug-ins)")
    parser.add_argument("--list-ports", action="store_true",
                         help="List available MIDI ports and exit")
    args = parser.parse_args()

    if args.list_ports:
        print("Outputs:", mido.get_output_names())
        print("Inputs:", mido.get_input_names())
        return

    if not args.files:
        sys.exit("No files given to push.")

    out_name = find_port(mido.get_output_names(), args.port, "output")
    in_names = mido.get_input_names()
    in_name = None
    try:
        in_name = find_port(in_names, args.port, "input")
    except SystemExit:
        print("warning: no matching MIDI input port found; replies will not be shown", file=sys.stderr)

    with mido.open_output(out_name) as outport:
        inport = mido.open_input(in_name) if in_name else None
        try:
            print(f"Waking disting NT on '{out_name}' (SysEx ID {args.sysex_id})")
            send(outport, inport, wake_message(args.sysex_id))
            time.sleep(0.05)

            ensure_directories(outport, inport, args.sysex_id, args.dest_dir)

            for local_path in args.files:
                remote_path = args.dest_dir.rstrip("/") + "/" + os.path.basename(local_path)
                upload_file(outport, inport, args.sysex_id, local_path, remote_path)

            print("Rescanning plugins on device")
            send(outport, inport, rescan_message(args.sysex_id))
        finally:
            if inport is not None:
                inport.close()

    print("Done.")


if __name__ == "__main__":
    main()
