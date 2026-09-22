---
layout: archive
title: "Lab 8: Stunts!"
permalink: /mae4190/lab8/
author_profile: false
---

{% include base_path %}

## Goal

I chose the drift stunt. The robot starts a few meters from the wall, drives forward fast, turns 180 degrees when it gets within `914 mm`, then drives back past the start line. The lab asked for repeated video evidence and timestamped plots of sensor data, KF output, and motor commands.

## Control Design

My first version reused the Lab 7 KF wall approach controller too literally. The robot estimated distance well, but then it tried to stop at one exact wall distance before turning. That made the car hunt back and forth near the wall, which broke the stunt rhythm and wasted time.

<div style="width:100%; max-width:700px;">
  <video preload="none" width="700" controls>
    <source src="/images/mae4190/lab8/IMG_3114.mp4" type="video/mp4">
  </video>
  <div style="text-align:center; font-size:0.95em;">Initial baseline. The robot keeps adjusting distance before the turn instead of drifting through the stunt.</div>
</div>

I changed the state machine after that. The KF is still used during the approach because it gives a fast distance estimate between ToF readings, but once the estimated distance crosses `914 mm`, the robot immediately switches to the Lab 6 yaw PID and turns to a heading that is `180 degrees` away from the current heading. It no longer chases a precise stop distance. After the turn finishes, it drives back with a small heading hold. The final drift parameters were `APPROACH_PWM = 160`, `RETURN_PWM = 160`, `ORIENT_KP = 2.5`, `ORIENT_KI = 0`, `ORIENT_KD = 0.05`, and `RETURN_YAW_KP = 1.0`.

The KF only matters in the approach stage. It predicts distance between slow ToF readings and decides when the robot should begin the stunt. The `180 degree` accuracy itself comes from the IMU orientation PID. I also made the turn exit condition explicit. The robot only leaves the rotation state after the turn progress exceeds about `150 degrees` and the heading error stays inside an `8 degree` band for `3` consecutive checks. That kept the stunt fast while preventing the robot from leaving the turn too early.

## Code

<details>
<summary>Arduino: direct-turn drift state machine</summary>
<div markdown="1">

```cpp
void handle_drift() {
    if (runMode != RUN_DRIFT) return;

    unsigned long now = millis();
    if (now - drift_start_ms > drift_timeout_ms) {
        motorsStop();
        runMode = RUN_IDLE;
        tx_characteristic_string.writeValue("DRIFT_TIMEOUT");
        return;
    }

    if (drift_phase == 0) {
        motorsForward(drift_approach_pwm);
        update_imu_state();

        float est_dist = 0.0f;
        int raw_mm = -1;
        if (!drift_update_distance_estimate(now, est_dist, raw_mm)) return;
        float control_dist = (raw_mm > 0) ? (float)raw_mm : est_dist;

        update_kf_control_input(drift_approach_pwm);

        if (control_dist <= drift_trigger_dist) {
            float heading_deg = 0.0f, gyro_dps = 0.0f;
            unsigned long heading_ts = now;
            if (!update_drift_heading_state(heading_deg, gyro_dps, heading_ts)) return;
            drift_approach_heading_ref = heading_deg;
            orient_target_deg = wrap_angle_deg(drift_approach_heading_ref + 180.0f);
            kf_last_u = 0.0f;
            reset_orient_pid_state(heading_ts, false);
            drift_phase = 1;
        }
        return;
    }

    if (drift_phase == 1) {
        float heading_deg = 0.0f, gyro_dps = imu_last_gyr_z;
        unsigned long heading_ts = now;
        if (!update_drift_heading_state(heading_deg, gyro_dps, heading_ts)) return;

        int pwm = 0;
        float err = 0.0f;
        if (!step_orient_pid_with_heading(heading_deg, err, pwm, heading_ts, false)) return;

        float turn_progress = fabsf(wrap_angle_deg(heading_deg - drift_approach_heading_ref));
        bool rotate_done = turn_progress >= DRIFT_TURN_PROGRESS_MIN &&
                           fabsf(err) <= DRIFT_ROTATE_DONE_BAND_DEG;

        if (rotate_done) drift_rotate_done_count++;
        else             drift_rotate_done_count = 0;

        if (drift_rotate_done_count >= DRIFT_ROTATE_DONE_COUNT) {
            motorsStop();
            drift_return_start_ms = heading_ts;
            drift_phase = 2;
        }
        return;
    }

    if (drift_phase == 2) {
        float heading_deg = 0.0f, gyro_dps = imu_last_gyr_z;
        unsigned long heading_ts = now;
        if (!update_drift_heading_state(heading_deg, gyro_dps, heading_ts)) return;

        float heading_err = wrap_angle_deg(orient_target_deg - heading_deg);
        int steer_bias = (int)lroundf(drift_return_yaw_kp * heading_err);
        steer_bias = constrain(steer_bias, -DRIFT_RETURN_STEER_MAX, DRIFT_RETURN_STEER_MAX);
        motorsForwardSteered(drift_return_pwm, steer_bias);

        if (now - drift_return_start_ms >= drift_return_ms) {
            motorsStop();
            drift_phase = 3;
            runMode = RUN_IDLE;
            tx_characteristic_string.writeValue("DRIFT_DONE");
        }
    }
}
```

</div>
</details>

<details>
<summary>Python: configure the stunt and collect BLE data</summary>
<div markdown="1">

```python
APPROACH_PWM  = 160
RETURN_PWM    = 160
TRIGGER_DIST  = 914
RETURN_MS     = 2500
TIMEOUT_MS    = 10000
RETURN_YAW_KP = 1.0

ble.send_command(CMD.SET_KF_PARAMS,
    f"{KF_D}|{KF_M}|{KF_S1}|{KF_S2}|{KF_S3}|{APPROACH_PWM}")
ble.send_command(CMD.SET_ORIENT_GAINS,
    f"{ORIENT_KP}|{ORIENT_KI}|{ORIENT_KD}")
ble.send_command(CMD.SET_DRIFT_PARAMS,
    f"{APPROACH_PWM}|{RETURN_PWM}|{TRIGGER_DIST}|{STOP_DIST}|"
    f"{RETURN_MS}|{TIMEOUT_MS}|{RETURN_YAW_KP}")

raw_messages = []
_drift_done = False

def drift_notify_handler(uuid, bytearray_data):
    global _drift_done
    msg = ble.bytearray_to_string(bytearray_data).strip()
    raw_messages.append(msg)
    if msg.startswith('DRF_END'):
        _drift_done = True

ble.start_notify(ble.uuid['RX_STRING'], drift_notify_handler)
ble.send_command(CMD.DRIFT_START,
    f"{APPROACH_PWM}|{RETURN_PWM}|{TRIGGER_DIST}|{STOP_DIST}|"
    f"{RETURN_MS}|{TIMEOUT_MS}|{RETURN_YAW_KP}")

time.sleep(TIMEOUT_MS / 1000 + 1.5)
ble.send_command(CMD.GET_DRIFT_DATA, '')
while not _drift_done:
    time.sleep(0.1)
```

</div>
</details>

## Results

The four videos document the full iteration path. The first video is the old baseline that still used distance adjustment before turning. The last three videos are the redesigned direct-turn controller, so they are the repeated evidence for the lab. The final two taped-tire runs are the best successful demonstrations because they complete the whole stunt smoothly and return past the starting line.

| Run | Time | Main behavior | Outcome |
|---|---:|---|---|
| Baseline logic | 12.73 s | Forward and backward adjustment before turn | Too slow and not continuous |
| Direct-turn before tire change | 7.19 s | Correct turn amount, but visible pause from friction | Better logic, but not smooth enough |
| Final run 1 | 5.44 s | Continuous drift with small lateral slip | Successful |
| Final run 2 | 4.29 s | Fastest continuous run with small lateral slip | Successful |

The first run with the new direct-turn logic fixed the old forward and backward adjustment, but the tire and floor friction was high enough that the robot hesitated during the last part of the turn. The motors still forced the chassis to the commanded heading, so the turn controller was correct, but the turn was not smooth and the run still took `7.19 s`.

<div style="width:100%; max-width:700px;">
  <video preload="none" width="700" controls>
    <source src="/images/mae4190/lab8/IMG_3117.mp4" type="video/mp4">
  </video>
  <div style="text-align:center; font-size:0.95em;">Direct-turn logic without the tire modification. The robot turns the right amount, but the high friction causes a visible pause.</div>
</div>

<img loading="lazy" decoding="async" src="/images/mae4190/lab8/lab8_drift.webp" width="700">
<div style="text-align:center; font-size:0.95em;">Sensor and control plot for the first direct-turn run.</div>

To reduce that sticking effect, I wrapped electrical tape around the outer surface of the tires. That lowered the tire-floor friction enough to make the turn much smoother.

<img loading="lazy" decoding="async" src="/images/mae4190/lab8/IMG_3125.webp" width="700">

<div style="text-align:center; font-size:0.95em;">Electrical tape added around the tire surface to reduce turning friction.</div>

After that, the next two runs looked much better. The approach, turn, and return flow together with no visible pause. Their clip lengths were `5.44 s` and `4.29 s`, which is much faster than both the old baseline and the untaped direct-turn version.

The tradeoff is that the lower friction causes some chassis drift during braking and rotation. More friction gave a cleaner stop location, but it made the turn sticky. Less friction made the stunt faster and smoother, but it introduced small lateral drift. The robot still turns the correct amount, but it does not always pivot around exactly the same point on the floor. It may slide a little left or right before coming back. I am okay with that trade because the stunt goal is a fast continuous drift and return, not a perfect zero radius spin in place. The three drift plots below show the raw ToF, KF estimate, heading, heading error, gyro rate, and motor command versus time. In the later runs, the return segment starts only after the heading trace reaches the full turn target, so the `180 degree` rotation is still enforced by the PID controller rather than by luck.

<div style="display:flex; gap:12px; flex-wrap:wrap; align-items:flex-start;">
  <div style="width:49%; text-align:center;">
    <video preload="none" width="100%" controls>
      <source src="/images/mae4190/lab8/IMG_3122.mp4" type="video/mp4">
    </video>
    <div style="font-size:0.95em;">Final run after the tire change. The whole stunt is continuous and much smoother.</div>
  </div>
  <div style="width:49%; text-align:center;">
    <video preload="none" width="100%" controls>
      <source src="/images/mae4190/lab8/IMG_3126.mp4" type="video/mp4">
    </video>
    <div style="font-size:0.95em;">Another final run. This one is the fastest clip and still returns cleanly.</div>
  </div>
</div>

<img loading="lazy" decoding="async" src="/images/mae4190/lab8/lab8_drift_1.webp" width="700">
<div style="text-align:center; font-size:0.95em;">Sensor and control plot for the first taped-tire run.</div>

<img loading="lazy" decoding="async" src="/images/mae4190/lab8/lab8_drift_2.webp" width="700">
<div style="text-align:center; font-size:0.95em;">Sensor and control plot for the fastest taped-tire run.</div>

Meet my cat Mulberry! 🐱

<img loading="lazy" decoding="async" src="/images/mae4190/cats/cat19.webp" width="400">
<img loading="lazy" decoding="async" src="/images/mae4190/cats/cat20.webp" width="400">

[Back to MAE 4190](/mae4190/)
