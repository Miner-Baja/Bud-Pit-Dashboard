import serial
import serial.tools.list_ports
import threading
import time
import json
from shared_buffer import rolling_buffer

serial_inst = None
serial_thread_running = False

log_filename = "lora_serial_log.jsonl"


def start_serial_reader(port, baudrate=115200):
    global serial_inst, serial_thread_running

    if serial_thread_running:
        print("Serial reader already running")
        return

    try:
        serial_inst = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        serial_thread_running = True
        print(f"Connected to {port}")

    except serial.SerialException as e:
        print(f"FAILED to open {port}: {e}")
        return

    def serial_reader():
        global serial_thread_running
        while True:
            try:
                line = serial_inst.readline()
                if not line:
                    continue

                packet = line.decode("utf-8", errors="ignore").strip()

                if not packet.startswith("+RCV="):
                    continue

                json_start = packet.find("{")
                json_end = packet.rfind("}") + 1

                if json_start == -1 or json_end == -1:
                    continue

                json_str = packet[json_start:json_end]
                data = json.loads(json_str)

                required = ["speed", "rpm", "voltage", "satellites", "status"]
                if not all(k in data for k in required):
                    continue

                data["timestamp"] = time.time()
                rolling_buffer.append(data)

                with open(log_filename, "a") as f:
                    f.write(json.dumps(data) + "\n")

            except json.JSONDecodeError:
                continue
            except serial.SerialException as e:
                print(f"Serial disconnected: {e}")
                break
            except Exception as e:
                print(f"Unexpected error: {e}")
                continue

        serial_thread_running = False
        try:
            serial_inst.close()
        except Exception:
            pass

    threading.Thread(target=serial_reader, daemon=True).start()


def get_com_ports():
    return [p.device for p in serial.tools.list_ports.comports()]
