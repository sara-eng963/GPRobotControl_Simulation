#!/usr/bin/env python3
"""Stage C2: source-anchored Classic CAN wire-time model; NOT hardware timing."""
import argparse
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
PDO = ROOT / "ServoDrive/AvatarM/avatar_m_pdo.c"
MASTER = ROOT / "CANComm/CANopen/canopen_master.c"
HEADER = ROOT / "CANComm/CANopen/canopen_master.h"

def bits(dlc, stuffed):
    if not 0 <= dlc <= 8:
        raise ValueError("Classic CAN DLC must be 0..8")
    # 11-bit CAN data, SOF..EOF plus 3 intermission bits = 47+8*DLC.
    # SOF through 15-bit CRC sequence can contain stuff bits: 34+8*DLC.
    return 47 + 8*dlc + ((33 + 8*dlc)//4 if stuffed else 0)

def source_contract():
    source = PDO.read_text()
    for name, payload in [
        ("avatar_m_build_rpdo4", 4),
        ("avatar_m_build_tpdo4", 6),
        ("avatar_m_build_sync", 0),
    ]:
        signature = r"(?:bool|void)\s+" + name + r"\s*\([^)]*\)\s*\{(.*?)\n\}"
        matches = re.findall(signature, source, re.DOTALL)
        if len(matches) != 1 or not re.search(
            r"frame->dlc\s*=\s*" + str(payload) + r"U\s*;", matches[0]
        ):
            raise ValueError("PDO source changed: " + name)
    master = MASTER.read_text()
    block = master.split("bool canopen_master_send_target_cycle(", 1)[1].split(
        "const AvatarMDrive *canopen_master_drive(", 1)[0]
    if block.count("avatar_m_drive_build_rpdo4_target(") != 1 or \
       block.count("canopen_master_send_sync(master)") != 1 or \
       block.index("avatar_m_drive_build_rpdo4_target(") > \
       block.index("canopen_master_send_sync(master)"):
        raise ValueError("CAN target cycle changed")
    header = HEADER.read_text()
    if not re.search(r"#define\s+CANOPEN_CONTROLLER_HEARTBEAT_MS\s+100U", header):
        raise ValueError("controller heartbeat no longer 100ms")
    if not re.search(r"#define\s+CANOPEN_MASTER_MAX_RX_PER_POLL\s+32U", header):
        raise ValueError("RX service bound changed")

def compute():
    source_contract()
    node_count = 6
    nominal = 6*bits(4, False)+6*bits(6, False)+bits(0, False)
    stuffed = 6*bits(4, True)+6*bits(6, True)+bits(0, True)
    hb_burst = (node_count+1)*bits(1, True)
    return {
        "basis": "standard 11-bit Classical CAN, 1 Mbit/s, 6 motors, 2ms",
        "source_frame_set": "6 RPDO4 DLC4 + 6 TPDO4 DLC6 + 1 SYNC DLC0",
        "nominal_cycle_bits": nominal,
        "worst_case_stuffed_cycle_bits": stuffed,
        "nominal_percent_of_2ms": 100*nominal/2000,
        "stuffed_percent_of_2ms": 100*stuffed/2000,
        "heartbeat_burst_bits": hb_burst,
        "stuffed_plus_7_heartbeat_bits": stuffed+hb_burst,
        "stuffed_plus_heartbeat_percent_of_2ms": 100*(stuffed+hb_burst)/2000,
        "stuffed_cycle_headroom_us": 2000-stuffed,
        "stuffed_plus_heartbeat_headroom_us": 2000-stuffed-hb_burst,
        "assumptions": [
          "One TPDO4 from every drive per 2ms",
          "Six motor heartbeat intervals not defined by this model; coincident burst assumes seven heartbeat frames total",
          "100ms heartbeat for controller is source-defined",
        ],
        "excluded": [
          "SDO/EMCY/NMT, arbitration from any additional devices",
          "transmission errors/retries, FDCAN TX FIFO queuing, motor response latency",
          "MCU CPU execution/ISR delay, drive jitter, real WCET"
        ],
    }

def selftest():
    assert bits(0, False)==47 and bits(8, False)==111
    assert bits(4, True)==95 and bits(6, True)==115
    p=compute()
    assert p["nominal_cycle_bits"]==1091
    assert p["worst_case_stuffed_cycle_bits"]==1315
    assert p["heartbeat_burst_bits"]==455
    assert p["stuffed_plus_7_heartbeat_bits"]==1770
    print("PASS: Stage C2 CAN source/DLC/worst-stuffing selftest")

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--out",type=Path)
    parser.add_argument("--self-test",action="store_true")
    args=parser.parse_args()
    if args.self_test:
        selftest()
        return
    if not args.out:
        parser.error("--out required")
    x=compute()
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/"can_wire_budget.json").write_text(json.dumps(x,indent=2)+"\n")
    lines=[
        "# Stage C2 1 Mbps six-node 500 Hz CANopen model","",
        "**Analytical, NOT a measured physical throughput/latency acceptance.**","",
        "Frame lengths are source-anchored in ServoDrive/AvatarM/avatar_m_pdo.c and CANComm/CANopen/canopen_master.c.",
        "11-bit data frame base: 47+8*DLC bits incl. intermission. Stuffable SOF through CRC-sequence region: 34+8*DLC bits.",
        "Conservative stuff bound: floor((33+8*DLC)/4).","",
        "| Traffic | Bits per 2ms | Wire time at 1Mbps | Utilization |",
        "|---|---:|---:|---:|",
        "| Cyclic 6 RPDO4 + 6 TPDO4 + SYNC, no stuffing | "+str(x["nominal_cycle_bits"])+" | "+str(x["nominal_cycle_bits"])+" us | "+str(x["nominal_percent_of_2ms"])+"% |",
        "| Same cyclic frames, worst-case bit stuffing | "+str(x["worst_case_stuffed_cycle_bits"])+" | "+str(x["worst_case_stuffed_cycle_bits"])+" us | "+str(x["stuffed_percent_of_2ms"])+"% |",
        "| Stuffed cyclic frames plus 7 coincident heartbeats | "+str(x["stuffed_plus_7_heartbeat_bits"])+" | "+str(x["stuffed_plus_7_heartbeat_bits"])+" us | "+str(x["stuffed_plus_heartbeat_percent_of_2ms"])+"% |",
        "",
        "**Caution:** aligned motor heartbeats are only a stress scenario; their actual periods are not fixed by this model.",
        "Model omits SDO, EMCY, NMT, errors, retransmission, hardware/software queue delay and motor response latency.",
        "**A 2ms frame-count feasibility estimate is NOT proof of 500Hz controller deadlines.**",
    ]
    (args.out/"can_wire_budget.md").write_text("\n".join(lines)+"\n")
    print("PASS: Stage C2 source-anchored CAN wire budget computed")
    print("Cyclic wire-time envelope: "+str(x["nominal_cycle_bits"])+".."+str(x["worst_case_stuffed_cycle_bits"])+" us/2ms")
    print("Aligned 7-heartbeat burst: "+str(x["stuffed_plus_7_heartbeat_bits"])+" us/2ms (SDO/retry unmodeled)")

if __name__=="__main__":
    try:
        main()
    except (OSError,ValueError,AssertionError) as e:
        print("FAIL: C2 CAN model:",e,file=sys.stderr)
        sys.exit(1)
