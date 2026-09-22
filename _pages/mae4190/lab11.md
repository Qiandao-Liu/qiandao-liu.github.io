---
layout: archive
title: "Lab 11: Localization (real)"
permalink: /mae4190/lab11/
author_profile: false
---

{% include base_path %}

## Overview

The goal of this lab was to run the Bayes filter on the real robot. On hardware I used only the update step from a 360 degree observation loop. The main issue was perceptual aliasing on the +x side of the map: several cells produce similar range signatures, so a uniform-prior update can jump to the wrong place.

## Simulation check

I first ran the provided simulation notebook to verify that the reference Bayes filter behavior was still correct before moving to BLE and real sensors.

<img loading="lazy" decoding="async" src='/images/mae4190/lab11/sim.webp' width='700'>

The final plot looked normal, so later failures on the real robot were much more likely to come from sensing and map ambiguity than from the filter math itself. I also ran a longer simulated trajectory and confirmed that the Bayes-filter estimate kept following ground truth better than raw odometry.

<img loading="lazy" decoding="async" src='/images/mae4190/lab11/traj.webp' width='700'>

## Observation loop on the real robot

The lab expects 18 readings, 20 degrees apart, starting at the current heading and rotating counter-clockwise. Instead of writing a new scan routine, I reused my Lab 9 map scan. The firmware performs a dense 3 degree sweep over about one full turn, then Python downsamples those samples into the 18 observations expected by the localization notebook. I used the right ToF for the final observation vector because its placement gives cleaner geometry during an in-place turn, and I subtracted a small sensor mount offset in Python to better match the map ray cast.

<details>
<summary>Python: perform_observation_loop using MAP scan</summary>
<div markdown="1">

```python
def _parse_map_status(self, msg):
    parts = msg.split('|')
    if len(parts) != 9 or parts[0] != 'MAP_STATUS':
        raise ValueError(f'Unexpected MAP_STATUS message: {msg}')
    return {
        'state': parts[1],
        'sample_idx': int(parts[2]),
        'log_pos': int(parts[3]),
        'phase': int(parts[4]),
        'start_ms': int(parts[5]),
        'now_ms': int(parts[6]),
        'sweep_deg': int(parts[7]),
        'step_deg': int(parts[8]),
    }

def perform_observation_loop(self, rot_vel=120):
    SENSOR_YAW_DEG = -90.0
    SENSOR_MOUNT_OFFSET_M = 0.03

    observation_count = int(self.config_params['mapper']['observations_count'])
    target_bearing_step = 360.0 / observation_count

    fw_step_deg = 3
    fw_sweep_deg = 380
    fw_samples_goal = int(np.ceil(fw_sweep_deg / fw_step_deg))
    timeout_ms = 120000
    turn_dir = -1

    map_buf = []
    map_done = {'value': False}

    def map_notify_handler(uuid, bytearray_data):
        msg = self.ble.bytearray_to_string(bytearray_data).strip()
        map_buf.append(msg)
        if msg.startswith('MAP_END|'):
            map_done['value'] = True

    self.ble.start_notify(self.ble.uuid['RX_STRING'], map_notify_handler)
    self.ble.send_command(
        CMD.MAP_START,
        f'{fw_step_deg}|{fw_samples_goal}|{timeout_ms}|{turn_dir}|{fw_sweep_deg}'
    )
    wait_deadline = time.time() + timeout_ms / 1000.0 + 3.0

    while time.time() < wait_deadline:
        if any(msg.startswith('MAP_DONE|') or msg.startswith('MAP_TIMEOUT') for msg in map_buf):
            break
        self.ble.send_command(CMD.GET_MAP_STATUS, '')
        try:
            last_status = self._parse_map_status(
                self.ble.receive_string(self.ble.uuid['RX_STRING'])
            )
            if last_status['state'] == 'IDLE':
                break
        except Exception:
            pass
        time.sleep(0.25)

    self.ble.send_command(CMD.GET_MAP_DATA, '')
    while not map_done['value']:
        time.sleep(0.10)

    rows = []
    for msg in map_buf:
        if not msg.startswith('MAP|'):
            continue
        parts = msg.split('|')
        rows.append({
            'target_deg': int(parts[1]) / 10.0,
            'heading_deg': int(parts[2]) / 10.0,
            'front_mm': int(parts[3]),
            'right_mm': int(parts[4]),
            'time_ms': int(parts[5]),
        })

    rows = sorted(rows, key=lambda row: row['time_ms'])
    headings_cw = np.array([row['heading_deg'] for row in rows], dtype=float)
    headings_unwrapped = np.rad2deg(np.unwrap(np.deg2rad(headings_cw)))
    delta = headings_unwrapped - headings_unwrapped[0]
    rotation_sign = 1.0 if delta[-1] >= 0 else -1.0
    position_mod = np.mod(rotation_sign * delta, 360.0)

    target_robot_positions = np.mod(
        np.arange(observation_count) * target_bearing_step - SENSOR_YAW_DEG,
        360.0,
    )

    right_vals = np.array([r['right_mm'] for r in rows], dtype=float)
    right_ok_mask = right_vals > 0
    selected_indices = []
    for target in target_robot_positions:
        diff = position_mod - target
        diff = (diff + 180.0) % 360.0 - 180.0
        order = np.argsort(np.abs(diff))
        j = next(int(k) for k in order if right_ok_mask[k])
        selected_indices.append(j)

    selected_rows = [rows[j] for j in selected_indices]
    right_mm = np.array([r['right_mm'] for r in selected_rows], dtype=float)
    sensor_ranges = np.clip(right_mm / 1000.0 - SENSOR_MOUNT_OFFSET_M, 0.0, None)[np.newaxis].T
    sensor_bearings = (np.arange(observation_count) * target_bearing_step)[np.newaxis].T
    return sensor_ranges, sensor_bearings
```

</div>
</details>

<details>
<summary>Arduino: map command entrypoints</summary>
<div markdown="1">

```cpp
case MAP_START:
{
    int step_deg = 3;
    int samples_goal = 127;
    int timeout_ms = 120000;
    int turn_dir = 1;
    int sweep_deg = 380;
    robot_cmd.get_next_value(step_deg);
    robot_cmd.get_next_value(samples_goal);
    robot_cmd.get_next_value(timeout_ms);
    robot_cmd.get_next_value(turn_dir);
    robot_cmd.get_next_value(sweep_deg);

    map_step_deg = constrain(step_deg, 1, 180);
    map_samples_goal = constrain(samples_goal, 1, MAP_LOG_LEN);
    map_timeout_ms = (unsigned long)max(1000, timeout_ms);
    map_turn_dir = (turn_dir >= 0) ? 1 : -1;
    map_sweep_deg = constrain(sweep_deg, 1, 720);
    start_map_run();
    break;
}

case GET_MAP_DATA:
    stream_map_history();
    break;

case GET_MAP_STATUS:
    send_map_status();
    break;
```

</div>
</details>

<details>
<summary>Arduino: map logging and status pipeline</summary>
<div markdown="1">

```cpp
void stream_map_history() {
    for (int i = 0; i < map_log_pos; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("MAP|");
        tx_estring_value.append((int)map_target_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)map_heading_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)map_front_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)map_right_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)map_t_hist[i]);
        ble_write_reliable_fast(tx_estring_value.c_str());
    }

    tx_estring_value.clear();
    tx_estring_value.append("MAP_END|");
    tx_estring_value.append(map_log_pos);
    ble_write_reliable_fast(tx_estring_value.c_str());
}

void send_map_status() {
    tx_estring_value.clear();
    tx_estring_value.append("MAP_STATUS|");
    tx_estring_value.append(runMode == RUN_MAP ? "RUNNING" : "IDLE");
    tx_estring_value.append("|");
    tx_estring_value.append(map_sample_idx);
    tx_estring_value.append("|");
    tx_estring_value.append(map_log_pos);
    tx_estring_value.append("|");
    tx_estring_value.append(map_phase);
    tx_estring_value.append("|");
    tx_estring_value.append((int)map_start_ms);
    tx_estring_value.append("|");
    tx_estring_value.append((int)millis());
    tx_estring_value.append("|");
    tx_estring_value.append(map_sweep_deg);
    tx_estring_value.append("|");
    tx_estring_value.append(map_step_deg);
    ble_write_reliable_fast(tx_estring_value.c_str());
}

if (map_phase == 1) {
    motorsStop();

    int front_mm = -1;
    int right_mm = -1;
    unsigned long tof_ts = heading_ts;
    if (read_map_tof_sample(front_mm, right_mm, tof_ts) && front_mm > 0) {
        finish_map_sample(heading_deg, front_mm, right_mm, tof_ts);
        return;
    }

    if (now - map_sample_wait_start_ms >= MAP_SAMPLE_WAIT_MS) {
        finish_map_sample(heading_deg, -1, -1, heading_ts);
    }
}
```

</div>
</details>

## Baseline update-only localization

I first ran the localization exactly as assigned: one update step from a uniform prior. That baseline showed a clear split. The left and center-left side of the map localized very well, while the +x side was the weak spot. The failure mode was repeatable: the robot sometimes matched a left-side pose because several right-side observations looked too similar under a completely uniform prior.

For the required poses, the baseline behavior was:

- `(-3, -2)`: stable and correct
- `(0, 3)`: stable and correct
- `(5, -3)`: sometimes aliased to a left-side pose, mismatched to (5, 3).
- `(5, 3)`: usually correct, but still had occasional left-side confusion, sometimes mismatched to (5, -3)

So the baseline already answered the main lab question: the robot does localize better in some poses, and the difference comes from how distinctive the local wall and corner geometry is.

## Prior-guided localization

To remove that aliasing, I added an `APPROX_POSE` prior before the update step. If the car starts from a previously high-confidence pose, then odometry is still good enough to estimate the rough region where the robot should be now, even if it is not accurate enough to run a full prediction step. I used that estimate only as a weak spatial prior, then let the observation update choose the final cell.

<details>
<summary>Python: weak approximate-pose prior</summary>
<div markdown="1">

```python
RESET_TO_UNIFORM = False
APPROX_POSE = (5, -3, None)
PRIOR_XY_SIGMA = 0.35
PRIOR_THETA_SIGMA = None

def _normalize_distribution(dist, label):
    total = float(np.sum(dist))
    if not np.isfinite(total) or total <= 0:
        raise RuntimeError(f'{label} collapsed; check prior/sensor settings')
    return dist / total

def _apply_pose_prior(localizer, approx_pose, xy_sigma, theta_sigma=None):
    prior = np.array(localizer.bel, dtype=float, copy=True)
    prior = _normalize_distribution(prior, 'incoming prior')

    if approx_pose is not None:
        x0, y0, theta0 = approx_pose
        dx = localizer.mapper.x_values - x0
        dy = localizer.mapper.y_values - y0
        spatial = np.exp(-0.5 * (dx * dx + dy * dy) / (xy_sigma * xy_sigma))
        prior *= spatial

    prior = _normalize_distribution(prior, 'real-robot prior')
    localizer.bel = prior.copy()
    localizer.bel_bar = prior.copy()
    return prior

run_prior = _apply_pose_prior(loc, APPROX_POSE, PRIOR_XY_SIGMA, PRIOR_THETA_SIGMA)
```

</div>
</details>

After adding that weak prior, all four required poses localized to the correct cell in all three trials (`3/3` each). The extra `(0,0)` validation point was also `3/3`. This was the version I carried into Lab 12.

### Pose `(-3, -2)`

This point localized very nicely even with the uniform-prior baseline. The final belief stayed in the correct cell and was essentially on top of the ground-truth pose, with no meaningful visible error. I think this point works so well because it is mostly enclosed by nearby walls with one main opening, so the scan is quite distinctive.

<div style="display:flex; gap:16px; align-items:flex-start; flex-wrap:wrap;">
  <div style="flex:1; min-width:320px; text-align:center;">
    <video preload="none" width="100%" controls>
      <source src='/images/mae4190/lab11/-3_-2.mp4' type='video/mp4'>
    </video>
    <div style="font-size:0.95em;">Run at (-3,-2,0).</div>
  </div>
  <div style="flex:1; min-width:320px; text-align:center;">
    <img loading="lazy" decoding="async" src='/images/mae4190/lab11/-3_-2.webp' width='100%'>
    <div style="font-size:0.95em;">Final belief for (-3,-2,0).</div>
  </div>
</div>

### Pose `(0, 3)`

This point also localized cleanly. The final belief was again in the correct cell and the error was negligible by eye. I think this pose is easier because it sees a strong corner plus additional structure deeper in the map, so the 360 degree scan is less symmetric than the difficult +x poses.

<div style="display:flex; gap:16px; align-items:flex-start; flex-wrap:wrap;">
  <div style="flex:1; min-width:320px; text-align:center;">
    <video preload="none" width="100%" controls>
      <source src='/images/mae4190/lab11/0_3.mp4' type='video/mp4'>
    </video>
    <div style="font-size:0.95em;">Run at (0,3,0).</div>
  </div>
  <div style="flex:1; min-width:320px; text-align:center;">
    <img loading="lazy" decoding="async" src='/images/mae4190/lab11/0_3.webp' width='100%'>
    <div style="font-size:0.95em;">Final belief for (0,3,0).</div>
  </div>
</div>

### Pose `(5, -3)`

This was the worst required point in the baseline. Under a uniform prior it could collapse onto the wrong left-side pose because the scan was not globally distinctive enough. After adding `APPROX_POSE`, it localized to the correct cell in all three trials. The remaining bias was small but visible: the estimate sat a little low and right of ground truth, with about `0.5 ft` of position error. My best explanation is that when the robot is near a tight corner, the ToF reading is slightly nonlinear and exaggerates how close that corner is, so the filter prefers a point that is a bit nearer to both walls than the real pose.

<div style="display:flex; gap:16px; align-items:flex-start; flex-wrap:wrap;">
  <div style="flex:1; min-width:320px; text-align:center;">
    <video preload="none" width="100%" controls>
      <source src='/images/mae4190/lab11/5_-3.mp4' type='video/mp4'>
    </video>
    <div style="font-size:0.95em;">Run at (5,-3,0) after adding the weak prior.</div>
  </div>
  <div style="flex:1; min-width:320px; text-align:center;">
    <img loading="lazy" decoding="async" src='/images/mae4190/lab11/5_-3.webp' width='100%'>
    <div style="font-size:0.95em;">Final belief for (5,-3,0).</div>
  </div>
</div>

### Pose `(5, 3)`

This point was better than `(5,-3)` in the baseline, but it could still alias occasionally. With the weak prior, all three trials ended in the correct cell. The residual bias here was larger than at `(5,-3)`: the estimate was a bit low and left of ground truth, with about `1 ft` of error. I suspect the same close-corner ToF exaggeration is involved here too, since the filter again behaves as if the robot is nearer to the nearby walls than it actually is.

<div style="display:flex; gap:16px; align-items:flex-start; flex-wrap:wrap;">
  <div style="flex:1; min-width:320px; text-align:center;">
    <video preload="none" width="100%" controls>
      <source src='/images/mae4190/lab11/5_3.mp4' type='video/mp4'>
    </video>
    <div style="font-size:0.95em;">Run at (5,3,0).</div>
  </div>
  <div style="flex:1; min-width:320px; text-align:center;">
    <img loading="lazy" decoding="async" src='/images/mae4190/lab11/5_3.webp' width='100%'>
    <div style="font-size:0.95em;">Final belief for (5,3,0).</div>
  </div>
</div>

I also tested `(0,0)` as an extra validation point. It was the easiest one: the surrounding walls are very distinctive, and all three runs were essentially exact since this place is so unique.

<div style="display:flex; gap:16px; align-items:flex-start; flex-wrap:wrap;">
  <div style="flex:1; min-width:320px; text-align:center;">
    <video preload="none" width="100%" controls>
      <source src='/images/mae4190/lab11/0_0.mp4' type='video/mp4'>
    </video>
    <div style="font-size:0.95em;">Extra validation run at (0,0,0).</div>
  </div>
  <div style="flex:1; min-width:320px; text-align:center;">
    <img loading="lazy" decoding="async" src='/images/mae4190/lab11/0_0.webp' width='100%'>
    <div style="font-size:0.95em;">Final belief for (0,0,0).</div>
  </div>
</div>

## Discussion

The baseline result showed the main lesson of this lab clearly: a single update step with a uniform prior is not always enough on the real robot, even when the Bayes filter code is correct. The robot localized best at `(-3,-2)`, `(0,3)`, and the extra `(0,0)` point because those places have distinctive nearby walls and openings. The right-side poses are more ambiguous, so multiple cells can produce similar 360 degree signatures.

The `APPROX_POSE` method fixed the wrong-cell jumps without pretending odometry is globally accurate. I did not use odometry as a full motion-model prediction step; I only used it to apply a weak prior over the plausible region before the update. That was enough to suppress the aliased left-side candidates. After that change, all four required poses were `3/3` correct-cell, and the only remaining errors were smaller sub-cell biases on the two right-side poses. That is the approach I will reuse in Lab 12.

Meet my cat Mulberry! 🐱

<img loading="lazy" decoding="async" src='/images/mae4190/cats/cat5.webp' width='300'> <img loading="lazy" decoding="async" src='/images/mae4190/cats/cat12.webp' width='300'>

[Back to MAE 4190](/mae4190/)
