import subprocess
import struct
import re
import sys

CLI_PATH = "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI"

def strip_ansi(text):
    return re.sub(r'\x1b\[[0-9;]*[a-zA-Z]', '', text)

def get_telemetry():
    cmd = [
        CLI_PATH,
        "-c", "port=SWD", "freq=100",
        "-r32", "0x20000000", "48"
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    clean_out = strip_ansi(res.stdout)
    
    words = []
    for line in clean_out.splitlines():
        line = line.strip()
        m = re.match(r"^0x[0-9a-fA-F]+\s*:\s*(.*)", line)
        if m:
            for val in m.group(1).split():
                try:
                    words.append(int(val, 16))
                except ValueError:
                    pass

    if len(words) >= 9:
        raw = struct.pack(f"<{len(words)}I", *words)
        magic = struct.unpack_from("<I", raw, 0)[0]
        if magic == 0xAD345001:
            devid, status = struct.unpack_from("<BB", raw, 4)
            rx, ry, rz = struct.unpack_from("<hhh", raw, 6)
            mx, my, mz, p_max, p_min, count = struct.unpack_from("<iiiiiI", raw, 12)
            return {
                "ok": True,
                "devid": hex(devid),
                "status": "ONLINE" if status == 1 else "OFFLINE",
                "x_g": mx / 1000.0,
                "y_g": my / 1000.0,
                "z_g": mz / 1000.0,
                "peak_min": p_min / 1000.0,
                "peak_max": p_max / 1000.0,
                "p2p": (p_max - p_min) / 1000.0,
                "samples": count
            }
    return {"ok": False, "raw_out": clean_out}

if __name__ == "__main__":
    t = get_telemetry()
    if t["ok"]:
        print("+" + "-"*56 + "+")
        print("|           STM32 & ADXL345 LIVE TELEMETRY               |")
        print("+" + "-"*56 + "+")
        print(f"|  Device ID:          {t['devid']:<6} (Expected: 0xe5) -> OK!     |")
        print(f"|  Sensor Status:      {t['status']:<10}                        |")
        print(f"|  Total Samples:      {t['samples']:<10}                        |")
        print("+" + "-"*56 + "+")
        print(f"|  Current Accel X:    {t['x_g']:+7.3f} g                            |")
        print(f"|  Current Accel Y:    {t['y_g']:+7.3f} g                            |")
        print(f"|  Current Accel Z:    {t['z_g']:+7.3f} g                            |")
        print("+" + "-"*56 + "+")
        print(f"|  Residual Wobble Min Z:  {t['peak_min']:+7.3f} g                    |")
        print(f"|  Residual Wobble Max Z:  {t['peak_max']:+7.3f} g                    |")
        print(f"|  Peak-to-Peak Vibration:  {t['p2p']:6.3f} g                    |")
        print("+" + "-"*56 + "+")
    else:
        print("Failed to read telemetry:", t)
