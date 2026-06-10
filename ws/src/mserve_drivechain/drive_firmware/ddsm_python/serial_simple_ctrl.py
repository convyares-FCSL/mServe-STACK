import argparse
import json
import sys
import threading
import time

import serial
from serial import SerialException, SerialTimeoutException


PRINT_LOCK = threading.Lock()


def log(message):
    timestamp = time.strftime("%H:%M:%S")
    with PRINT_LOCK:
        print(f"[{timestamp}] {message}", flush=True)


def decode_bytes(data):
    return data.decode("utf-8", errors="replace")


def serial_line_state(ser):
    fields = {
        "open": ser.is_open,
        "dtr": ser.dtr,
        "rts": ser.rts,
    }

    for name in ("cts", "dsr", "cd", "ri", "in_waiting", "out_waiting"):
        try:
            fields[name] = getattr(ser, name)
        except (OSError, SerialException):
            fields[name] = "unavailable"

    return ", ".join(f"{key}={value}" for key, value in fields.items())


def open_serial(port, baudrate, timeout, write_timeout):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    ser.timeout = timeout
    ser.write_timeout = write_timeout
    ser.rtscts = False
    ser.dsrdtr = False
    ser.dtr = False
    ser.rts = False

    log(f"Opening {port} at {baudrate} baud with DTR=False RTS=False")
    ser.open()

    # Some Windows USB serial drivers briefly assert these during open anyway.
    # Set them low again immediately and report the resulting state.
    ser.dtr = False
    ser.rts = False
    log(f"Opened {port}: {serial_line_state(ser)}")
    return ser


def read_serial(ser, stop_event, status, partial_idle):
    buffer = bytearray()
    partial_report_len = 0
    last_rx_at = None
    status["alive"] = True

    try:
        while not stop_event.is_set():
            try:
                waiting = ser.in_waiting
                data = ser.read(waiting or 1)
            except (OSError, SerialException) as exc:
                status["error"] = repr(exc)
                log(f"RX ERROR: {exc!r}")
                break

            now = time.monotonic()
            if not data:
                if (
                    buffer
                    and last_rx_at is not None
                    and len(buffer) != partial_report_len
                    and now - last_rx_at >= partial_idle
                ):
                    log(
                        "RX partial/no newline "
                        f"({len(buffer)} bytes): {decode_bytes(buffer)!r}"
                    )
                    partial_report_len = len(buffer)
                continue

            status["last_rx_at"] = now
            last_rx_at = now
            buffer.extend(data)

            while b"\n" in buffer:
                line, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)
                partial_report_len = 0
                log(f"RX line ({len(line) + 1} bytes): {decode_bytes(line).rstrip()}")

        if buffer:
            log(f"RX trailing ({len(buffer)} bytes): {decode_bytes(buffer)!r}")
    finally:
        status["alive"] = False


def is_json(command):
    try:
        json.loads(command)
        return True
    except json.JSONDecodeError as exc:
        log(f"TX ERROR: input is not valid JSON at char {exc.pos}: {exc.msg}")
        if '"' not in command and command.startswith("{") and command.endswith("}"):
            log(r"PowerShell may have stripped quotes. Try: --probe '{\"T\":10405}'")
        return False


def send_command(ser, command):
    command = command.strip()
    if not command:
        return False

    if not is_json(command):
        return False

    payload = command.encode("utf-8") + b"\n"

    try:
        written = ser.write(payload)
        ser.flush()
    except SerialTimeoutException as exc:
        log(f"TX TIMEOUT: {exc!r}")
        return False
    except (OSError, SerialException) as exc:
        log(f"TX ERROR: {exc!r}")
        return False

    log(f"TX wrote {written}/{len(payload)} bytes: {payload!r}")
    if written != len(payload):
        log("TX WARNING: pyserial accepted fewer bytes than requested")
        return False
    return True


def print_status(ser, reader_thread, status):
    reader_state = "alive" if reader_thread.is_alive() else "stopped"
    log(f"Reader thread: {reader_state}")
    if status.get("error"):
        log(f"Reader error: {status['error']}")
    log(f"Serial state: {serial_line_state(ser)}")


def main():
    parser = argparse.ArgumentParser(description="Diagnostic Serial JSON Communication")
    parser.add_argument(
        "port",
        nargs="?",
        default="COM5",
        help="Serial port name. Defaults to COM5.",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate.")
    parser.add_argument(
        "--boot-wait",
        type=float,
        default=3.0,
        help="Seconds to wait after open for ESP32 boot/reset output.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.1,
        help="Serial read timeout in seconds.",
    )
    parser.add_argument(
        "--write-timeout",
        type=float,
        default=2.0,
        help="Serial write timeout in seconds.",
    )
    parser.add_argument(
        "--partial-idle",
        type=float,
        default=0.35,
        help="Seconds of idle time before printing unterminated received bytes.",
    )
    parser.add_argument(
        "--probe",
        help='Optional JSON command to send after boot wait, e.g. \'{"T":10405}\'.',
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Exit after --probe and --response-wait seconds.",
    )
    parser.add_argument(
        "--response-wait",
        type=float,
        default=2.0,
        help="Seconds to wait for responses before exiting with --once.",
    )
    args = parser.parse_args()

    if args.once and not args.probe:
        parser.error("--once requires --probe")

    ser = None
    stop_event = threading.Event()
    status = {"alive": False, "error": None, "last_rx_at": None}

    try:
        ser = open_serial(args.port, args.baud, args.timeout, args.write_timeout)

        reader_thread = threading.Thread(
            target=read_serial,
            args=(ser, stop_event, status, args.partial_idle),
            name="serial-reader",
            daemon=True,
        )
        reader_thread.start()

        log(f"Waiting {args.boot_wait:.1f}s for ESP32 boot/reset output")
        time.sleep(args.boot_wait)
        print_status(ser, reader_thread, status)

        if args.probe:
            send_command(ser, args.probe)
            if args.once:
                time.sleep(args.response_wait)
                print_status(ser, reader_thread, status)
                return 0

        log("Enter JSON commands. Use :status to inspect the port, :quit to exit.")
        while True:
            try:
                command = input("json> ")
            except EOFError:
                break

            if command.strip().lower() in {":q", ":quit", "quit", "exit"}:
                break
            if command.strip().lower() == ":status":
                print_status(ser, reader_thread, status)
                continue

            if not reader_thread.is_alive():
                print_status(ser, reader_thread, status)
                log("Reader is stopped; writes may fail if the USB serial handle broke.")

            send_command(ser, command)
    except KeyboardInterrupt:
        log("Interrupted")
    except SerialException as exc:
        log(f"OPEN ERROR: {exc!r}")
        return 1
    finally:
        stop_event.set()
        if ser is not None and ser.is_open:
            ser.close()
            log("Serial port closed")

    return 0


if __name__ == "__main__":
    sys.exit(main())
