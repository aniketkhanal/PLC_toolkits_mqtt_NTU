import snap7
from snap7.util import set_int, set_real, set_bool, get_bool, get_int
import time
import argparse
import yaml
import os
import sys
import select
import json
import psycopg2
import subprocess
import requests


# ============================================================================
#  WORKING setpoint method (the key difference from v4):
#  The chiller setpoint is the SM 1234 analog current output QW112 (0..20 mA).
#  The PLC does NOT drive this channel in this setup, so a direct snap7 write
#  to QW112 HOLDS and the chiller follows it -- no DB468/ACT_DATA/START needed.
#
#  Calibration (2 measured points, chiller display vs QW112 counts):
#      20736 counts (15.00 mA) -> -5 C
#      23736 counts (17.17 mA) -> +5 C   => 300 counts / C
#      count = 20736 + (T + 5) * 300 ,  clamped to 0..27648 (0..20 mA)
#  Channel max 27648 ~ +18 C, so warm targets above ~+18 C CLIP.
#  0 counts ~ -80 C (the power-off floor).
#
#  NOTE: this bypasses the PLC interlocks (dew point, door, chiller faults) and
#  the value PERSISTS if this script dies -- run attended.
# ============================================================================
SETPOINT_CH = 112          # QW112 = chiller setpoint (analog out Ch0)
REF_COUNT = 20736          # counts at REF_TEMP
REF_TEMP = -5.0            # degC at REF_COUNT
COUNTS_PER_C = 300.0
RAW_MIN, RAW_MAX = 0, 27648


def temp_to_count(t):
    raw = round(REF_COUNT + (t - REF_TEMP) * COUNTS_PER_C)
    return max(RAW_MIN, min(RAW_MAX, raw))


def count_to_temp(raw):
    return REF_TEMP + (raw - REF_COUNT) / COUNTS_PER_C


def set_analog_setpoint(client, temp, retries=4):
    """
    Drive the chiller setpoint directly via QW112. Re-writes if the value doesn't
    stick (handles a transient PLC scan / stale read), and only warns if it
    PERSISTENTLY won't hold -- which would mean the PLC is really driving the channel.
    """
    raw = temp_to_count(temp)
    eff = count_to_temp(raw)
    if abs(eff - temp) > 0.6:
        print(f"  [SETPOINT] {temp:+.1f}°C is OUT OF RANGE -> clipped to {eff:+.1f}°C "
              f"(QW112={raw}, ~{raw / 27648 * 20:.2f} mA)")
    else:
        print(f"  [SETPOINT] chiller setpoint -> {temp:+.1f}°C "
              f"(QW112={raw}, ~{raw / 27648 * 20:.2f} mA)")
    d = bytearray(2)
    set_int(d, 0, raw)
    back = None
    for _ in range(retries):
        client.ab_write(SETPOINT_CH, d)
        time.sleep(0.5)
        back = get_int(client.ab_read(SETPOINT_CH, 2), 0)
        if abs(back - raw) < 50:
            return eff  # held
    print(f"  [WARNING] QW112 won't hold ({back} vs {raw}) after {retries} tries -- "
          f"the PLC may be driving the channel. Cooling will not work until it's freed.")
    return eff


def load_config(config_path: str) -> dict:
    if not os.path.exists(config_path):
        print(f"Error, there is no configuration file: '{config_path}'")
        sys.exit(1)
    with open(config_path, 'r', encoding='utf-8') as f:
        try:
            return yaml.safe_load(f)
        except yaml.YAMLError as exc:
            print(f"Error: {exc}")
            sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description='PLC Thermal Cycle Controller (direct-analog setpoint)')
    parser.add_argument('-c', '--config', default='Control_HMI.yml', help='Path to the YAML config file')
    parser.add_argument('-f', '--force-run', action='store_true', help='Override safety mode checks')
    parser.add_argument('-b', '--batch', default=None,
                         help="batch_name of the existing mmts_batch_logging row for this run "
                              "(the same batch_name the MMTS GUI already logged). If given, this "
                              "script UPDATEs that row's cycle_count as real low points are hit. "
                              "If omitted, cycle counts are not written to the database.")
    parser.add_argument('--cycles', type=int, default=None,
                         help="Override experiment.cycles from the config file (e.g. from the "
                              "GUI's Number of Cycles field). If omitted, uses the config file's value.")
    parser.add_argument('--mmts-gui-path', default=None, help='Path to MultiModuleTeststandUI, needed to trigger IV scans automatically')
    parser.add_argument('--mmts-gui-port', type=int, default=5001, help='Port the Flask GUI is listening on.')
    parser.add_argument('--auto-iv', action='store_true', help='Trigger IV scans automatically instead of waiting or typed "continue".')
    return parser.parse_args()


def create_client(plc_config):
    client = snap7.client.Client()
    try:
        client.connect(plc_config['ip'], plc_config['rack'], plc_config['slot'])
        return client
    except Exception as e:
        print(f"PLC connection failed: {e}")
        return None


def read_sensor_real(client, db_number, offset):
    try:
        data = client.db_read(db_number, offset, 4)
        return snap7.util.get_real(data, 0)
    except Exception:
        return 0.0


def get_filtered_sensor_data(client, db_number):
    """
    Reads all 8 PT1000 sensors, filters out any that read 0.0 (broken),
    and returns a list of valid temperatures along with their computed average.
    """
    # Offsets directly from the PDF compilation sheet
    # pt_offsets = [44, 82, 120, 158, 196, 234, 272]  # Extended up to your array block
    pt_offsets = [82, 120, 158, 196, 234]

    valid_temps = []
    for idx, offset in enumerate(pt_offsets, 1):
        temp = read_sensor_real(client, db_number, offset)
        # Skip dead or unconfigured channels showing exactly 0.0 or faulty extreme limits
        if temp != 0.0 and temp > -50.0:
            valid_temps.append(temp)
        else:
            print(f"    [WARNING] PT Sensor #{idx} at Offset {offset} is reading 0.0 or Faulty. Skipping...")

    if not valid_temps:
        return [20.0], 20.0  # Emergency fallback to prevent division by zero

    avg_temp = sum(valid_temps) / len(valid_temps)
    return valid_temps, avg_temp


def write_temp_setpoint(client, db_number, offset, value):
    data = bytearray(4)
    set_real(data, 0, value)
    client.db_write(db_number, offset, data)


def write_int_value(client, db_number, offset, value):
    data = bytearray(2)
    set_int(data, 0, value)
    client.db_write(db_number, offset, data)


# ============================================================================
#  SAFETY INTERLOCKS (replicated in software).
#  We only bypass the SETPOINT output (QW112); every safety INPUT the PLC uses
#  is still readable over snap7, so we enforce the same protections here. These
#  are active while this script runs -- run attended.
# ============================================================================
SAFE_AMBIENT = 20.0
DEWPOINT_OFFSETS = (314, 356)   # DB15 REALs, the two dew-point sensors
# Two complementary dew-point protections:
#  1) START GATE -- don't BEGIN cooling unless the box is already dry enough, i.e. the
#     dew point is at or below START_DEWPOINT_MAX. Checked once before each cool-down.
#  2) RUNTIME SAFEGUARD -- the COLDEST measured surface must stay at least
#     CONDENSATION_MARGIN above the current dew point at ALL times, or the run STOPS.
#     Checked continuously on ACTUAL temperature -- this is the dry-air-failure safety net.
START_DEWPOINT_MAX = -38.0      # (1) start gate: refuse to begin cooling if dew point > this
CONDENSATION_MARGIN = 10.0      # (2) runtime: stop if coldest sensor < dewpoint + margin
DEWPOINT_VALID_MAX = 30.0       # readings above this are non-physical -> faulty channel
DEWPOINT_VALID_MIN = -100.0     # below this = faulty/unscaled channel
DOOR_LOCK_M = (60, 0)           # %M60.0 = door locked (the bit system_status() gates on)
CHILLER_FAULT_BITS = ((446, 0), (446, 1))  # DB15 FIELD_FAULT / CH_FAULT


def read_m_bool(client, byte, bit):
    return get_bool(client.mb_read(byte, 1), 0, bit)


def read_db_bool(client, db_num, byte, bit):
    return get_bool(client.db_read(db_num, byte, 1), 0, bit)


def measured_dewpoint(client, db_num):
    """
    Worst-case (highest) of the VALID dew-point sensors. Discards non-physical
    readings from faulty/disconnected channels (e.g. the +38.5°C ghost reading).
    Returns None if NO sensor is valid -> caller must fail safe (refuse to cool).
    """
    valid = [v for v in (read_sensor_real(client, db_num, off) for off in DEWPOINT_OFFSETS)
             if DEWPOINT_VALID_MIN < v < DEWPOINT_VALID_MAX and v != 0.0]
    if not valid:
        return None
    return max(valid)   # most humid reading = most conservative


class SafetyAbort(Exception):
    """Raised when an interlock trips; the setpoint is driven safe first."""

class CondensationRisk(Exception):
    """Coldest sensor too close to the dew point; hold and retry."""

def enforce_safety(client, db_num, target=None, min_actual_temp=None):
    """
    Software interlocks. On any violation, drive the setpoint back to ambient and
    raise SafetyAbort. Always checks door + chiller faults + dew-point validity, plus:
      - START GATE (pass target=...): refuse to begin cooling if dew point > START_DEWPOINT_MAX.
      - RUNTIME SAFEGUARD (pass min_actual_temp=...): hold if the coldest measured surface
        is within CONDENSATION_MARGIN of the dew point (dry-air-failure protection).
    """
    # Collect EVERY tripped interlock so the abort message shows the full picture
    # (not just whichever one happened to be checked first).
    reasons = []
    if not read_m_bool(client, *DOOR_LOCK_M):
        reasons.append("DOOR not locked (%M60.0=False)")
    # NOTE: CHILLER_FIELD_FAULT (DB.446.0) is NOT gated on -- it was observed True with
    # no alarm on the chiller panel (and is False only while the chiller is running), so
    # it misreports and would deadlock the start gate. CH_FAULT (446.1) is still trusted.
    if read_db_bool(client, db_num, *CHILLER_FAULT_BITS[1]):
        reasons.append("CHILLER_CH_FAULT (DB.446.1)")

    # Dew-point checks only matter when a cold target / actual temp is in play.
    condensation_reason = None
    if target is not None or min_actual_temp is not None:
        dp = measured_dewpoint(client, db_num)
        if dp is None:
            reasons.append("no valid dew-point reading (both sensors faulty)")
        else:
            if target is not None and dp > START_DEWPOINT_MAX:
                reasons.append(f"DEW POINT {dp:+.1f}°C is above the {START_DEWPOINT_MAX:+.1f}°C "
                               f"start requirement (box not dry enough to begin cooling)")
            if min_actual_temp is not None and min_actual_temp < dp + CONDENSATION_MARGIN:
                condensation_reason = (f"CONDENSATION RISK: coldest sensor {min_actual_temp:+.1f}°C is "
                               f"within {CONDENSATION_MARGIN:.0f}°C of the dew point {dp:+.1f}°C")

    if reasons:
        for r in reasons:
            print(f"\n  [SAFETY ABORT] {r}")
        set_analog_setpoint(client, SAFE_AMBIENT)
        raise SafetyAbort("; ".join(reasons))

    if condensation_reason:
        print(f"\n [CONDENSATION HOLD] {condensation_reason}")
        raise CondensationRisk(condensation_reason)

def monitored_sleep(client, db_num, seconds):
    """time.sleep(seconds) but keeps the safety interlocks live (polls ~every 10s)."""
    waited = 0.0
    while waited < seconds:
        valid_temps, _ = get_filtered_sensor_data(client, db_num)
        try:
            enforce_safety(client, db_num, min_actual_temp=min(valid_temps))
        except CondensationRisk as e:
            print(f"\n  [CONDENSATION HOLD] {e} -- holding at {COLD_HOLD_SETPOINT}°C, soak timer paused")
            set_analog_setpoint(client, COLD_HOLD_SETPOINT)
            time.sleep(20)
            continue
        step = min(10.0, seconds - waited)
        time.sleep(step)
        waited += step

def fetch_module_ids_from_gui(port):
    try:
        resp = requests.get(f'http://127.0.0.1:{port}/api/get_config', timeout=5)
        resp.raise_for_status()
        return resp.json()  # {"1L": "<serial>", "1C": "<serial>", ...}
    except requests.exceptions.RequestException as e:
        print(f'[IV_SCAN] Could not reach MMTS GUI on port {port}: {e}')
        return {}

def run_automatic_iv_scan(mmts_gui_path, port, batch_name, iteration_label, target_temp):
    if not batch_name:
        print('[IV_SCAN] No --batch given; skipping automatic IV scan.')
        return
    mmts_gui_path = os.path.expanduser(mmts_gui_path)
    module_ids = fetch_module_ids_from_gui(port)
    if not module_ids:
        print('[IV_SCAN] No modules configured in the GUI; skipping scan.')
        return
    module_args = ' '.join(f'{pos}={serial}' for pos, serial in module_ids.items())
    cmd = (
            f'cd "{mmts_gui_path}" && source init_bash_vars.sh && source .venv/bin/activate && '
            f'make -f makefile_task3 run {module_args} '
            f"batch='{batch_name}' iteration='{iteration_label}' currentTEMPERATURE='{target_temp}'"
    )
    print(f'[IV_SCAN] Starting automatic IV scan ({iteration_label})...')
    subprocess.run(cmd, shell=True, executable='/bin/bash', check=False)
    print('[IV_SCAN] IV scan finished, resuming thermal cycling.')

def wait_for_iv_automatic(client, db_num, current_temp, mmts_gui_path, mmts_gui_port, batch_name, iteration_label):
    """
    Automatic counterpart to wait_for_iv_continue(): holts at the current setpoint, runs the IV scan against whatever 
    modules are configured in the GUI, then resumes.
    """
    print(f">>> TAKE IV CURVE (coldest sensor: {current_temp:+.1f}°C) -- running automatically")
    run_automatic_iv_scan(mmts_gui_path, mmts_gui_port, batch_name, iteration_label, current_temp)

def wait_for_iv_continue(client, db_num, current_temp):
    """
    Pause for the cold IV curve in cycle 1. Holds the chiller at its current cold
    setpoint and waits for the user to type 'continue'. Safety interlocks stay ACTIVE
    the whole time (door / fault / condensation), so a hazard still aborts to ambient.
    """
    print("\n" + "=" * 64)
    print(f">>> TAKE COLD IV CURVE  (current coldest sensor: {current_temp:+.1f}°C, cycle 1)")
    print(">>> The chiller is HOLDING at the cold setpoint.")
    print(">>> When the IV test is finished, type 'continue' and press Enter to resume.")
    print("=" * 64)
    while True:
        valid_temps, _ = get_filtered_sensor_data(client, db_num)
        try:
            enforce_safety(client, db_num, min_actual_temp=min(valid_temps))  # keep guarding during IV test
        except CondensationRisk as e:
            print(f"\n  [CONDENSATION HOLD] {e} -- holding at {COLD_HOLD_SETPOINT}°C")
            set_analog_setpoint(client, COLD_HOLD_SETPOINT)
            time.sleep(5.0)
            continue
        # wait up to 5s for input without blocking the safety poll
        ready, _, _ = select.select([sys.stdin], [], [], 5.0)
        if ready:
            line = sys.stdin.readline().strip().lower()
            if line == "continue":
                print(">>> Resuming automatic thermal cycling...\n")
                return
            print(">>> (type 'continue' once the IV test is done)")


# ============================================================================
#  PHASE TEMPERATURES (cold overdrive+settle, warm)
# ============================================================================
COLD_DRIVE_SETPOINT = -55.0   # command this WHILE cooling down (fast)
COLD_HOLD_SETPOINT = -40.0    # once cold, settle here for the soak / IV hold
COLD_DONE_AVG = -38.0         # cold phase complete when lowest-3 sensor avg <= this
COLD_DONE_MAX = -33.0         # ...and the warmest sensor <= this (whole-box uniformity)
WARM_SETPOINT = 18.0          # warm setpoint; ~+18C is the channel max (20 mA full scale)
WARM_DONE_MIN = 15.0          # warm phase complete when the COLDEST sensor reaches this

# ============================================================================
#  CHECKPOINT / RESUME
#  A run writes its position (cycle, phase) before each phase. On the next start,
#  if the file still exists the run was interrupted -> offer to resume. The file
#  is cleared only on full successful completion.
# ============================================================================
CHECKPOINT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'chiller_checkpoint.json')
PHASE_NAMES = ['cooling', 'warming']   # phase index 0 = cold phase, 1 = warm phase


def save_checkpoint(cycle, phase, total_cycles, batch_name, completed_cycles):
    try:
        with open(CHECKPOINT_FILE, 'w') as f:
            json.dump({'cycle': cycle, 'phase': phase, 'total_cycles': total_cycles,
                       'batch_name': batch_name, 'completed_cycles': completed_cycles,
                       'time': time.strftime('%Y-%m-%d %H:%M:%S')}, f)
    except Exception as e:
        print(f"  [WARN] could not write checkpoint: {e}")


def load_checkpoint():
    try:
        with open(CHECKPOINT_FILE) as f:
            return json.load(f)
    except Exception:
        return None


def clear_checkpoint():
    try:
        os.remove(CHECKPOINT_FILE)
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"  [WARN] could not clear checkpoint: {e}")


# ============================================================================
#  THERMAL CYCLE COUNT -> mmts_batch_logging (hgcdb)
#  This does NOT create a new row. It REPLACES the cycle_count on the EXISTING
#  row the MMTS GUI already logged for this batch (identified by --batch) with
#  the REAL number of completed cold-phase low points, since that column was
#  previously populated with an unrelated/incorrect value.
#  DB errors are non-fatal -- a flaky DB link must never abort a physical run.
# ============================================================================


def db_connect(db_cfg):
    try:
        return psycopg2.connect(host=db_cfg['DB_HOST'], port=db_cfg['DB_PORT'],
                                 user=db_cfg['DB_USER'], password=db_cfg['DB_PASSWORD'],
                                 dbname=db_cfg['DB_NAME'])
    except Exception as e:
        print(f"  [WARN] could not connect to database: {e}")
        return None


def update_cycle_count(db_cfg, batch_name, completed_cycles):
    """Replace cycle_count on the EXISTING mmts_batch_logging row for batch_name."""
    conn = db_connect(db_cfg)
    if conn is None:
        return
    try:
        with conn, conn.cursor() as cur:
            # Every row of this batch gets the count (the GUI logs several rows per
            # batch and some are cleaned up later) -- whichever row survives keeps it.
            cur.execute("UPDATE mmts_batch_logging SET cycle_count = %s WHERE batch_name = %s",
                        (completed_cycles, batch_name))
            if cur.rowcount == 0:
                print(f"  [WARN] no mmts_batch_logging row found for batch_name="
                      f"'{batch_name}' -- cycle_count was NOT recorded")
            else:
                print(f"  [DB] batch '{batch_name}': cycle_count = {completed_cycles} "
                      f"written to {cur.rowcount} row(s)")
    except Exception as e:
        print(f"  [WARN] could not update cycle count: {e}")
    finally:
        conn.close()


def do_cold_phase(client, db_num, exp_cfg, cycle, auto_iv, mmts_gui_path, mmts_gui_port, batch_name):
    """
    PHASE 1: overdrive cold -> settle to hold -> IV pause (cycle 1) / cold soak.
    Returns True if the cold target was genuinely reached (a real thermal-cycle low
    point), or False if the phase was instead forced to advance by the stall timeout.
    """
    print(f"\n[PHASE 1] Cooling fast (drive {COLD_DRIVE_SETPOINT}°C), "
          f"will settle to {COLD_HOLD_SETPOINT}°C once cold")
    enforce_safety(client, db_num, target=COLD_DRIVE_SETPOINT)  # START GATE: dew point <= -35
    set_analog_setpoint(client, COLD_DRIVE_SETPOINT)            # overdrive to cool fast

    print(f"--> Cooling down... Waiting until sensors reach the cold target "
          f"(warmest <= {COLD_DONE_MAX:+.1f}°C, lowest-3 avg <= {COLD_DONE_AVG:+.1f}°C)...")
    stall_counter = 0
    last_avg_temp = 999.0
    target_reached = False
    while True:
        valid_temps, current_avg = get_filtered_sensor_data(client, db_num)
        max_sensor_temp = max(valid_temps)
        try:
            enforce_safety(client, db_num, min_actual_temp=min(valid_temps))  # condensation guard
        except CondensationRisk as e:
            print(f"\n  [CONDENSATION HOLD] {e} -- holding at {COLD_HOLD_SETPOINT}°C instead of driving colder")
            set_analog_setpoint(client, COLD_HOLD_SETPOINT)
            time.sleep(20)
            continue
        lowest3_list = sorted(valid_temps)[:3]
        lowest3 = sum(lowest3_list) / len(lowest3_list)

        print(f"    [COOLING] Warmest Sensor: {max_sensor_temp:.2f}°C | "
              f"Group Avg: {current_avg:.2f}°C | Stall: {stall_counter}/90", end="\r")

        if (lowest3 <= COLD_DONE_AVG) and (max_sensor_temp <= COLD_DONE_MAX):
            print(f"\n[TARGET ACHIEVED] warmest sensor <= {COLD_DONE_MAX:+.1f}°C and "
                  f"lowest-3 avg <= {COLD_DONE_AVG:+.1f}°C")
            target_reached = True
            break
        
        if abs(last_avg_temp - current_avg) < 0.10:
            stall_counter += 1
            print('Stall =+1')
        else:
            stall_counter = 0
            print('Stall = 0')
            
        if stall_counter >= 90:  # 90 * 20s = 30 min stall threshold
            print(f"\n[STALL ALERT] System leveled out at {current_avg:.2f}°C. Advancing...")
            break
        last_avg_temp = current_avg
        time.sleep(20)

    # Settle: stop overdriving, hold at the cold setpoint (chiller ramps itself).
    print(f"--> Cold target reached. Settling chiller to hold {COLD_HOLD_SETPOINT}°C...")
    set_analog_setpoint(client, COLD_HOLD_SETPOINT)

    if (cycle == 1 or cycle == exp_cfg['cycles']):
        cold_valid, _ = get_filtered_sensor_data(client, db_num)
        if auto_iv:
            iteration_label = 'iteration_2' if cycle == 1 else 'iteration_3'
            wait_for_iv_automatic(client, db_num, min(cold_valid), mmts_gui_path, mmts_gui_port, batch_name, iteration_label)
        else:
            wait_for_iv_continue(client, db_num, min(cold_valid))
    else:
        print(f"--> Starting cold soak timer for {exp_cfg['idle_cold_min']} minutes...")
        monitored_sleep(client, db_num, exp_cfg['idle_cold_min'] * 60)

    return target_reached


def do_warm_phase(client, db_num, exp_cfg):
    """PHASE 2: drive warm -> hold until coldest sensor warm enough -> warm soak."""
    print(f"\n[PHASE 2] Driving Chiller to Warm Target: {WARM_SETPOINT}°C")
    enforce_safety(client, db_num)  # door / fault still apply
    set_analog_setpoint(client, WARM_SETPOINT)

    print(f"--> Heating up... Waiting until ALL working sensors reach {WARM_DONE_MIN:.1f}°C...")
    stall_counter = 0
    last_avg_temp = -999.0
    while True:
        valid_temps, current_avg = get_filtered_sensor_data(client, db_num)
        min_sensor_temp = min(valid_temps)
        try:
            enforce_safety(client, db_num, min_actual_temp=min_sensor_temp)  # door / fault / condensation
        except CondensationRisk as e:
            print(f"\n  [CONDENSATION HOLD] {e} -- pausing warm-up")
            time.sleep(20)
            continue

        print(f"    [HEATING] Coldest Sensor: {min_sensor_temp:.2f}°C | "
              f"Group Avg: {current_avg:.2f}°C | Stall: {stall_counter}/45", end="\r")

        if min_sensor_temp >= WARM_DONE_MIN:
            print(f"\n[TARGET ACHIEVED] All active PT1000 elements have reached >= {WARM_DONE_MIN:.1f}°C!")
            break

        if abs(current_avg - last_avg_temp) < 0.10:
            stall_counter += 1
        else:
            stall_counter = 0
        if stall_counter >= 45:
            print(f"\n[STALL ALERT] Heating leveled out at {current_avg:.2f}°C. Advancing...")
            break
        last_avg_temp = current_avg
        time.sleep(20)

    print(f"--> Starting warm soak timer for {exp_cfg['idle_warm_min']} minutes...")
    monitored_sleep(client, db_num, exp_cfg['idle_warm_min'] * 60)


def shutdown_system(client, db_num):
    print("\n[SHUTDOWN SEQUENCE INITIATED]")
    set_analog_setpoint(client, SAFE_AMBIENT)  # drive setpoint back toward ambient
    print("  [SUCCESS] System returning to ambient conditions.")


def main():
    args = parse_args()
    cfg = load_config(args.config)

    plc_cfg = cfg['plc']
    exp_cfg = cfg['experiment']
    db_cfg = cfg['database']
    db_num = plc_cfg['db_number']
    if args.cycles:
        exp_cfg['cycles'] = args.cycles
    total_cycles = exp_cfg['cycles']

    client = create_client(plc_cfg)
    if not client or not client.get_connected():
        print("PLC connection failed, abort!")
        return

    # --- Resume from an interrupted run? ---
    start_cycle, start_phase = 1, 0
    batch_name, completed_cycles = None, 0
    is_fresh_start = True
    ckpt = load_checkpoint()
    if ckpt:
        ck_cycle = ckpt.get('cycle', 1)
        ck_phase = ckpt.get('phase', 0)
        ck_total = ckpt.get('total_cycles', total_cycles)
        ck_name = PHASE_NAMES[ck_phase] if 0 <= ck_phase < len(PHASE_NAMES) else '?'
        print("\n[RESUME] An incomplete thermal cycle was detected:")
        print(f"  was on cycle {ck_cycle}/{ck_total}, '{ck_name}' phase "
              f"(saved {ckpt.get('time', '?')}).")
        ans = input("  Resume previous thermal cycle? [y/n]: ").strip().lower()
        if ans in ('y', 'yes'):
            start_cycle, start_phase, total_cycles = ck_cycle, ck_phase, ck_total
            batch_name = ckpt.get('batch_name')
            completed_cycles = ckpt.get('completed_cycles', 0)
            is_fresh_start = False
            print(f"  Resuming at cycle {start_cycle}, '{PHASE_NAMES[start_phase]}' phase.\n")
        else:
            print("  Starting a fresh run from cycle 1.\n")
            clear_checkpoint()

    # A fresh run (no resume, or resume declined) links to the batch_name the
    # MMTS GUI already logged for this job, if one was given via --batch.
    # A resumed checkpoint can carry batch_name='' (not None) from a prior run
    # that itself started with no --batch -- treat that the same as missing.
    if not batch_name:
        batch_name = args.batch
        completed_cycles = 0

    if batch_name:
        print(f"  [DB] cycle_count will replace the existing value on "
              f"mmts_batch_logging batch_name='{batch_name}'")
    else:
        print("  [DB] no --batch given -- cycle counts will NOT be written to the database")

    try:
        if args.auto_iv and is_fresh_start:
            _, current_avg = get_filtered_sensor_data(client, db_num)
            print("\n[IV_SCAN] Running initial room-temperature IV scan (iteration_1) before thermal cycling...")
            run_automatic_iv_scan(args.mmts_gui_path, args.mmts_gui_port, batch_name, 'iteration_1', current_avg)
        print(
            f"\n[STARTING THERMAL CYCLING] Profiling {total_cycles} Cycles between "
            f"{WARM_SETPOINT}°C and {COLD_HOLD_SETPOINT}°C (cold drive {COLD_DRIVE_SETPOINT}°C)")

        for cycle in range(start_cycle, total_cycles + 1):
            print(f"\n=========================================")
            print(f"       STARTING CYCLE {cycle} OF {total_cycles}       ")
            print(f"=========================================")

            # Resume only skips earlier phases of the FIRST resumed cycle.
            begin_phase = start_phase if cycle == start_cycle else 0
            for phase in range(begin_phase, len(PHASE_NAMES)):
                save_checkpoint(cycle, phase, total_cycles, batch_name, completed_cycles)
                if phase == 0:
                    if do_cold_phase(client, db_num, exp_cfg, cycle, args.auto_iv, args.mmts_gui_path, args.mmts_gui_port, batch_name):
                        completed_cycles += 1
                        print(f"  [THERMAL CYCLE] Low point reached -- "
                              f"completed cycles so far: {completed_cycles}")
                        if batch_name:
                            update_cycle_count(db_cfg, batch_name, completed_cycles)
                        save_checkpoint(cycle, phase, total_cycles, batch_name, completed_cycles)
                else:
                    do_warm_phase(client, db_num, exp_cfg)

            print(f"[CYCLE COMPLETE] Cycle {cycle} of {total_cycles} finished successfully.")

        clear_checkpoint()   # whole run finished cleanly -> no resume next time

        if args.auto_iv:
            _, current_avg = get_filtered_sensor_data(client, db_num)
            print("\n[IV_SCAN] Running final IV scan (iteration_4) after thermal cycling completes...")
            run_automatic_iv_scan(args.mmts_gui_path, args.mmts_gui_port, batch_name, 'iteration_4', current_avg)

        shutdown_system(client, db_num)

    except SafetyAbort as e:
        print(f"\n[ABORTED ON SAFETY INTERLOCK] {e}")
    except CondensationRisk as e:
        print(f"\n[UNEXPECTED CONDENSATION HOLD at top level] {e} -- returning to ambient as a precaution.")
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Returning chiller to ambient...")
    except Exception as e:
        print(f"Unexpected Runtime Exception: {e}")
    finally:
        # Safe-exit: never leave the chiller stuck at a cold setpoint on exit.
        try:
            if client and client.get_connected():
                set_analog_setpoint(client, SAFE_AMBIENT)
                print("[SAFE EXIT] Setpoint returned to ambient.")
        except Exception:
            pass
        client.disconnect()
        print('PLC Disconnected safely.')


if __name__ == "__main__":
    main()
