#!/usr/bin/env python3
"""
ZVD Input Shaping Live Terminal Monitor
Reads telemetry from STM32 SRAM via ST-LINK in real time.
Struct layout from zvd_main.c (packed, little-endian):
  Offset  0: uint32 magic (0xABCD1234)
  Offset  4: uint8  devid
  Offset  5: uint8  sensor_ok
  Offset  6: uint8  phase
  Offset  7: uint8  cycle
  Offset  8: uint32 nat_freq_mhz
  Offset 12: uint32 damp_ratio_ppm
  Offset 16: uint32 Td_ms
  Offset 20: uint32 ZV_A1_pct
  Offset 24: uint32 ZV_A2_pct
  Offset 28: uint32 ZV_t1_ms
  Offset 32: uint32 ZV_t2_ms
  Offset 36: uint32 ZVD_A1_pct
  Offset 40: uint32 ZVD_A2_pct
  Offset 44: uint32 ZVD_A3_pct
  Offset 48: uint32 ZVD_t1_ms
  Offset 52: uint32 ZVD_t2_ms
  Offset 56: uint32 ZVD_t3_ms
  Offset 60: int32  unshaped_p2p_mg
  Offset 64: int32  zv_p2p_mg
  Offset 68: int32  zvd_p2p_mg
  Offset 72: int32  curr_z_mg
  Offset 76: int32  curr_max_mg
  Offset 80: int32  curr_min_mg
  Offset 84: uint32 zv_reduction_x10
  Offset 88: uint32 zvd_reduction_x10
  Offset 92: uint32 total_samples
  Offset 96: uint32 zero_crossings
  Total: 100 bytes = 25 words
"""

import subprocess, struct, re, time, os

CLI = ("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/"
       "STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI")

PHASES = {0: "IDLE", 1: "UNSHAPED SNAP  >>", 2: "ZV SHAPED      >>",
          3: "ZVD SHAPED     >>", 4: "RETURNING HOME <<"}

R  = "\033[0m"
BD = "\033[1m"
RED = "\033[91m"
YEL = "\033[93m"
GRN = "\033[92m"
CYN = "\033[96m"

def strip_ansi(t):
    return re.sub(r'\x1b\[[0-9;]*[a-zA-Z]', '', t)

def read_sram():
    res = subprocess.run(
        [CLI, "-c", "port=SWD", "freq=100", "-r32", "0x20000000", "104"],
        capture_output=True, text=True, timeout=10
    )
    clean = strip_ansi(res.stdout)
    words = []
    for line in clean.splitlines():
        m = re.match(r"^\s*0x[0-9a-fA-F]+\s*:\s*(.*)", line)
        if m:
            for v in m.group(1).split():
                try: words.append(int(v, 16))
                except: pass

    if len(words) < 25:
        return None
    raw = struct.pack(f"<{len(words)}I", *words)

    magic = struct.unpack_from("<I", raw, 0)[0]
    if magic != 0xABCD1234:
        return None

    devid, sensor_ok, phase, cycle = struct.unpack_from("<BBBB", raw, 4)
    fn_mhz, damp_ppm, td_ms        = struct.unpack_from("<III",  raw, 8)
    zv_a1, zv_a2, zv_t1, zv_t2    = struct.unpack_from("<IIII", raw, 20)
    zvd_a1, zvd_a2, zvd_a3, zvd_t1, zvd_t2, zvd_t3 = struct.unpack_from("<IIIIII", raw, 36)
    us_p2p, zv_p2p, zvd_p2p, curr_z, curr_max, curr_min = struct.unpack_from("<iiiiii", raw, 60)
    zv_red, zvd_red, total_s, zero_c = struct.unpack_from("<IIII", raw, 84)

    return dict(
        devid=hex(devid), ok=sensor_ok, phase=phase, cycle=cycle,
        fn_hz=fn_mhz/1000.0, zeta=damp_ppm/1000000.0, td_ms=td_ms,
        zv_a1=zv_a1, zv_a2=zv_a2, zv_t2=zv_t2,
        zvd_a1=zvd_a1, zvd_a2=zvd_a2, zvd_a3=zvd_a3,
        zvd_t2=zvd_t2, zvd_t3=zvd_t3,
        us_p2p=us_p2p/1000.0, zv_p2p=zv_p2p/1000.0, zvd_p2p=zvd_p2p/1000.0,
        curr_z=curr_z/1000.0, curr_max=curr_max/1000.0, curr_min=curr_min/1000.0,
        zv_red=zv_red/10.0, zvd_red=zvd_red/10.0,
        samples=total_s, zero_c=zero_c
    )

def pbar(val, ref=10.0, w=30, color=RED):
    n = int(min(abs(val)/max(ref,0.01), 1.0)*w)
    return color + "#"*n + R + "-"*(w-n)

def divider(char="─", w=68):
    return f"  {char*w}"

if __name__ == "__main__":
    print("Connecting to STM32 via ST-LINK... (Ctrl+C to quit)\n")
    while True:
        try:
            d = read_sram()
            os.system("clear")
            ts = time.strftime("%H:%M:%S")
            print(f"{BD}{'═'*70}{R}")
            print(f"{BD}  ZVD INPUT SHAPING LIVE DASHBOARD        {CYN}[{ts}]{R}")
            print(f"{BD}{'═'*70}{R}")

            if not d:
                print(f"\n  {YEL}STM32 booting or waiting for first cycle...{R}\n")
                time.sleep(3)
                continue

            # Status bar
            ph = PHASES.get(d['phase'], "UNKNOWN")
            pc = {1:RED, 2:YEL, 3:GRN, 4:CYN}.get(d['phase'], "")
            s_ok = f"{GRN}ONLINE  devid={d['devid']}{R}" if d['ok'] else f"{RED}OFFLINE devid={d['devid']}{R}"
            print(f"  Sensor:   {s_ok}      Cycle: {BD}#{d['cycle']}{R}   Samples: {d['samples']}")
            print(f"  Phase:    {pc}{BD}{ph}{R}")
            print(f"  Live Z:   {d['curr_z']:+6.3f} g  (Range this stroke: {d['curr_min']:+.3f} g to {d['curr_max']:+.3f} g)")
            print(divider())

            # Beam ID
            print(f"\n{BD}  BEAM NATURAL FREQUENCY (Measured from Unshaped Residual){R}")
            print(divider("·"))
            if d['fn_hz'] > 0.1:
                print(f"  fn  = {BD}{d['fn_hz']:.2f} Hz{R}   (Zero crossings: {d['zero_c']} in 3s window)")
                print(f"  Td  = {d['td_ms']} ms        ζ = {d['zeta']:.4f} ({d['zeta']*100:.2f}%)")
            else:
                print(f"  {YEL}Waiting for first unshaped snap to measure frequency...{R}")

            # ZV params
            print(f"\n{BD}{YEL}  ZV SHAPER — 2 Impulses (Destructive Interference){R}")
            print(divider("·"))
            if d['fn_hz'] > 0.1:
                print(f"  t=0 ms    →  Impulse A1 = {d['zv_a1']:2d}% of total steps")
                print(f"  t={d['zv_t2']} ms  →  Impulse A2 = {d['zv_a2']:2d}% of total steps  (= Td/2, cancels vibration from A1)")
            else:
                print(f"  {YEL}Computed automatically after first unshaped snap.{R}")

            # ZVD params
            print(f"\n{BD}{GRN}  ZVD SHAPER — 3 Impulses (Robust, tolerates ±15% freq error){R}")
            print(divider("·"))
            if d['fn_hz'] > 0.1:
                tot = d['zvd_a1'] + d['zvd_a2'] + d['zvd_a3']
                print(f"  t=0 ms    →  Impulse A1 = {d['zvd_a1']:2d}%  |  A1+A2+A3 = {tot}%")
                print(f"  t={d['zvd_t2']} ms  →  Impulse A2 = {d['zvd_a2']:2d}%  (= Td/2)")
                print(f"  t={d['zvd_t3']} ms  →  Impulse A3 = {d['zvd_a3']:2d}%  (= Td)")
            else:
                print(f"  {YEL}Computed automatically after first unshaped snap.{R}")

            # Vibration comparison
            print(f"\n{BD}  VIBRATION COMPARISON — Peak-to-Peak Residual Acceleration{R}")
            print(divider())
            ref = max(d['us_p2p'], 0.05)
            print(f"  {RED}{BD}UNSHAPED{R}  {d['us_p2p']:6.3f} g  |{pbar(d['us_p2p'], ref, 30, RED)}|")
            print(f"  {YEL}{BD}ZV      {R}  {d['zv_p2p']:6.3f} g  |{pbar(d['zv_p2p'], ref, 30, YEL)}|  {YEL}↓ {d['zv_red']:.1f}%{R}")
            print(f"  {GRN}{BD}ZVD     {R}  {d['zvd_p2p']:6.3f} g  |{pbar(d['zvd_p2p'], ref, 30, GRN)}|  {GRN}↓ {d['zvd_red']:.1f}%{R}")
            print(divider())
            print(f"\n  Refreshing every 3s... Press {BD}Ctrl+C{R} to stop.\n")

        except KeyboardInterrupt:
            print("\nMonitor stopped.")
            break
        except Exception as e:
            print(f"Error: {e}")

        time.sleep(3)
