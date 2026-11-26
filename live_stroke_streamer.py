import subprocess
import struct
import re
import time
import sys

CLI_PATH = "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI"

def strip_ansi(text):
    return re.sub(r'\x1b\[[0-9;]*[a-zA-Z]', '', text)

def read_telemetry():
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

    if len(words) >= 8:
        raw = struct.pack(f"<{len(words)}I", *words)
        magic = struct.unpack_from("<I", raw, 0)[0]
        if magic == 0xAD345002:
            stroke_cnt = struct.unpack_from("<I", raw, 4)[0]
            devid, status = struct.unpack_from("<BB", raw, 8)
            curr_rz = struct.unpack_from("<h", raw, 10)[0]
            curr_mz, p_max, p_min, p2p, freq_x10, total_samples = struct.unpack_from("<iiiiiI", raw, 12)
            return {
                "ok": True,
                "stroke": stroke_cnt,
                "devid": hex(devid),
                "status": "ONLINE" if status == 1 else "INIT",
                "curr_z_g": curr_mz / 1000.0,
                "peak_min_g": p_min / 1000.0,
                "peak_max_g": p_max / 1000.0,
                "p2p_g": p2p / 1000.0,
                "freq_hz": freq_x10 / 10.0,
                "samples": total_samples
            }
    return {"ok": False}

if __name__ == "__main__":
    print("=" * 70)
    print("  REAL-TIME CONTINUOUS STROKE VIBRATION STREAMER")
    print("  Movement: 0° -> 40° (Sudden Snap) -> 40° -> 0° (Smooth Return)")
    print("=" * 70)
    
    last_stroke = -1
    # Run 4 stroke captures to show continuous live data
    for attempt in range(4):
        t = read_telemetry()
        if t["ok"]:
            print(f"\n>>> [STROKE #{t['stroke']}] Status: {t['status']} | Samples: {t['samples']}")
            print(f"    Current Z-Accel:          {t['curr_z_g']:+7.3f} g")
            print(f"    Wobble Peak Range:        [{t['peak_min_g']:+7.3f} g  to  {t['peak_max_g']:+7.3f} g]")
            print(f"    PEAK-TO-PEAK VIBRATION:    {t['p2p_g']:6.3f} g")
            print(f"    ESTIMATED NATURAL FREQ:    {t['freq_hz']:5.1f} Hz")
            
            # ASCII vibration intensity bar
            bars = int(min(max(t['p2p_g'] * 15, 1), 40))
            bar_graph = "#" * bars
            print(f"    Vibration Intensity:      |{bar_graph:<40}| ({t['p2p_g']:.2f}g)")
        else:
            print(f"Reading stroke data... (attempt {attempt+1})")
        time.sleep(1.2)
