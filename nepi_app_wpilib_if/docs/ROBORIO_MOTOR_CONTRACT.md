# RoboRIO Motor Command Contract

Audience: the WPILib developer writing the RoboRIO side. This document is
self-contained. You do not need to read any NEPI source to implement it.

This is the per-motor half of the NEPI-to-RoboRIO NetworkTables contract. An
operator moves a motor slider on the NEPI RBX page; NEPI writes a speed for one
motor; the RoboRIO drives that motor. The chassis-level commands (`/NEPI/RBX/Command`)
are covered in `ROBORIO_VELOCITY_CONTRACT.md` and are not repeated here.

The problem this contract solves: a motor command used to be written once.
If the link dropped while a motor was running, the RoboRIO kept the last value
and the motor kept spinning. NEPI now re-sends every running motor's command at
10 Hz, and the RoboRIO must stop any motor whose command goes quiet.

---

## Keys

One group per motor, under `/NEPI/MotorCommand/<motor_id>`.

| Key | Type | Direction | Meaning |
|---|---|---|---|
| `/NEPI/MotorCommand/<motor_id>/speed_ratio` | double | NEPI → RoboRIO | Commanded speed, 0.0 to 1.0. |
| `/NEPI/MotorCommand/<motor_id>/timestamp` | double | NEPI → RoboRIO | NEPI's clock at write time, seconds. Changes on every write. |

NEPI writes `speed_ratio` then `timestamp`, then flushes, on every write.

### What `motor_id` means

`motor_id` is the number the operator types into the **RoboRIO Id** box for a
motor slot, in the Motors section of the WPILib app on the NEPI RUI. NEPI does
not interpret it. It is not a CAN id, a PWM channel or a controller index unless
your code decides it is: **the RoboRIO owns the mapping from `motor_id` to a
physical controller** (which SPARK MAX, which CAN id, which PWM port).

Use the same ids you publish motor feedback under (`/NEPI/Motors/<motor_id>/...`),
so the operator sees the motor they are commanding.

A `motor_id` your code does not recognise must be ignored, not mapped to a
default motor.

### Range: 0.0 to 1.0, forward only

`speed_ratio` is a speed magnitude from 0.0 (stopped) to 1.0 (full output) in
the motor's forward direction. There is no reverse in this contract. NEPI clamps
to 0.0–1.0 before writing, so a correct NEPI never sends anything else.

If the RoboRIO reads a value outside 0.0–1.0 anyway (NaN, negative, above 1.0),
treat it as a fault: **set that motor to 0** and do not clamp it into range. A
negative value must never be passed to the controller as reverse.

What "forward" and "full output" mean for a given motor (inversion, current
limits, ramp rate, a cap below 100% duty cycle) is the RoboRIO's decision, the
same way the controller mapping is.

---

## The command lifecycle NEPI drives

For one motor slot:

1. The operator moves the slider. NEPI writes `speed_ratio` and a fresh
   `timestamp` once.
2. While `speed_ratio` is above 0, NEPI **re-writes the same value at 10 Hz**
   with a fresh `timestamp` each time. Only `timestamp` changes between
   re-writes.
3. When the motor is stopped, NEPI writes `speed_ratio` 0.0 **once**. A zero is
   never streamed: after one write, that key goes quiet.

NEPI writes the explicit 0.0 in step 3 when any of these happen:

- The operator sets the slider to 0.
- The operator presses Stop on the RBX page (see *STOP* below).
- The operator timeout expires (see below).
- The RBX device is disabled or rebuilt, or the WPILib app is disabled or shut
  down.
- The operator changes a slot's RoboRIO Id or reduces the slot count while that
  slot's motor is running. The 0.0 goes to the **old** `motor_id`, before the
  new mapping takes effect.

If the link is already down, that zero cannot arrive. That is the case the
watchdog exists for.

---

## Watchdog: required, not optional

**For each motor, the RoboRIO must set the motor to 0 if that motor's
`timestamp` key has not changed for 250 ms.**

This is a safety requirement, and it is the same 250 ms window as the
chassis-velocity watchdog, so one pattern serves both. The failure it prevents:
NEPI crashes, the radio drops or someone unplugs the Ethernet while a motor runs
at 0.8. Without a watchdog the RoboRIO holds 0.8 indefinitely.

At 10 Hz, 250 ms allows two missed writes before the motor stops.

### Measure on the RoboRIO's clock, not from the timestamp value

`timestamp` is NEPI's clock. The two clocks are not synchronised and may be
seconds or years apart, so **never compare the `timestamp` value with RoboRIO
time.** `Timer.getFPGATimestamp() - timestamp` is meaningless.

Use `timestamp` only as a change signal. When its value differs from the last
value you saw, record **your own** clock at that moment. The watchdog fires when
your own clock is 250 ms past that record. NEPI guarantees the value changes on
every write, so "changed" is a reliable proxy for "a write arrived".

### Per motor, not global

The watchdog is per `motor_id`. A quiet key stops that one motor only; other
motors with live streams keep running.

A motor whose last command was 0.0 is already stopped. Its key goes quiet
immediately after that write, and the watchdog firing on it is harmless.

### At startup

Treat a motor you have never seen a write for as stopped. Values left on
NetworkTables from an earlier NEPI session look like valid commands, but their
`timestamp` will not be changing, so the watchdog stops them 250 ms after you
first read them. Do not drive a motor on a value you have not seen change.

---

## STOP zeros every motor

The RBX Command Request group's STOP (`/NEPI/RBX/Command/command_type` = 4) is
a whole-robot stop. **When the RoboRIO receives a STOP request, it must set
every motor it drives to 0**, including motors running under a per-motor
command, not only the drivetrain.

NEPI sends both on an operator Stop: an explicit 0.0 to each running motor's
`speed_ratio`, then the STOP request. Either one alone must be enough to stop
those motors. A motor stopped by STOP stays stopped until a new `speed_ratio`
write arrives, meaning one with a timestamp change after the STOP.

---

## Operator timeout: NEPI-side only

The RBX page has a **Motor Test Timeout** box, the `motor_test_timeout_s`
setting: 1 to 300 seconds, default 30. If an operator leaves a motor running
and does not touch its slider for that long, NEPI stops the motor itself by
writing 0.0 once and logging a warning.

This timeout lives entirely in NEPI. The RoboRIO does not read it, does not need
to implement it and cannot see it. From the RoboRIO's side it is an ordinary
0.0 write. It is not a substitute for the watchdog: it runs on NEPI, so it
cannot fire if NEPI is the thing that failed.

---

## Known NEPI-side gap

The keepalive covers motor commands from the RBX page's manual motor control.
The WPILib app also exposes a standard NEPI Motors interface whose commands
write the same keys **once, with no keepalive**. Once this watchdog is in place,
a command arriving that way runs for 250 ms and then stops. That is safe but
not useful. Bringing that path onto the same stream is a NEPI change; nothing
on the RoboRIO side changes.

---

## Keys to implement, as a checklist

Read (NEPI writes, RoboRIO consumes):

```
/NEPI/MotorCommand/<motor_id>/speed_ratio    double   -- 0.0..1.0, forward only
/NEPI/MotorCommand/<motor_id>/timestamp      double   -- NEPI clock; change signal only
/NEPI/RBX/Command/command_type               int      -- 4 = STOP, zeros every motor
```

Behaviour, run every robot loop (20 ms):

```
For each motor_id this robot maps to a controller:
  ts = read /NEPI/MotorCommand/<motor_id>/timestamp
  if ts != last_ts[motor_id]:                  # a write arrived
      last_ts[motor_id]     = ts
      last_change[motor_id] = now()             # RoboRIO clock, NOT ts
      ratio = read /NEPI/MotorCommand/<motor_id>/speed_ratio
      if ratio is NaN or ratio < 0.0 or ratio > 1.0:
          ratio = 0.0                           # fault: stop, do not clamp
      commanded[motor_id] = ratio

  if never_seen(motor_id) or now() - last_change[motor_id] > 0.250:
      commanded[motor_id] = 0.0                 # watchdog

  controller_for(motor_id).set(commanded[motor_id])

On RBX Command Request with command_type 4 (STOP):
  for every motor_id: commanded[motor_id] = 0.0
```

`never_seen` is true until the first timestamp change after robot code starts,
so values left on NetworkTables from an earlier session never drive a motor.
