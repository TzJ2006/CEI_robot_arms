# Developer guide: Piper client and policy link

This guide is for people changing the code or checking the protocol. Operating steps are in `USER_GUIDE.md`. The Chinese version is `开发文档.md`.

```mermaid
flowchart TB
  subgraph pc [Windows or macOS]
    client[policy_link.py once]
    armclient[piper_client.py]
    client --> armclient
  end
  subgraph bus [CAN 1 Mbps]
    adapter[candleLight gs_usb]
  end
  subgraph cluster [Login node and compute node]
    tunnel[Login node ssh -L]
    server[Compute node serve_policy.py]
  end
  armclient --> adapter
  adapter --> piper[Piper]
  client -->|WebSocket msgpack| tunnel
  tunnel --> server
```

## Find a topic

| You want to change or look up | Section |
| --- | --- |
| Read-only state and gripper motion | [piper_client.py](#piper_clientpy) |
| Fields sent to the model | [Observation contract](#observation-contract) |
| How a returned chunk becomes pose commands | [Action contract](#action-contract) |
| WebSocket and timeouts | [Policy link](#policy-link) |
| The device will not open | [Device ownership](#device-ownership) |
| CAN frame layout and IDs | [Protocol](#protocol) |
| Libraries already shown not to work | [Do not switch to these entry points](#do-not-switch-to-these-entry-points) |
| Work that is not done | [Boundary](#boundary) |
| A candidate policy repository arrives | [Check a candidate repository](#check-a-candidate-repository) |

## Which files to use

| File | Role |
| --- | --- |
| `piper_client.py` | Opens candleLight (WinUSB on Windows, libusb on macOS), reads `0x2A1`–`0x2A8`, and sends gripper `0x159`, leave-teach `0x150`, enable `0x471`, MOVE P `0x151`, and end pose `0x152`–`0x154` |
| `policy_link.py` | Builds the observation, keeps one WebSocket open, and loops inference. The camera and CAN writes are not in the same process |
| `policy_step.py` | Integrates one action chunk into an end-effector trajectory in the base frame and writes it to CAN. It does not import `piper_sdk` at runtime |
| `team_config.json` | Login account, SSH alias, checkpoint, and log directory. This is the only file to edit when the environment changes |
| `team_config.py` | `run` submits the job, waits until it listens, opens one local forward, and starts continuous control. `once` starts another round while the forward is up. `down` cancels the job and closes the forward. `job` and `serve` still only print commands |
| `simple_policy_server.py` | Scripted policy server for testing the link without a model. See [macOS and the simple server](#macos-and-the-simple-server) |
| `USER_GUIDE.md` | Operating steps |
| `用户文档.md` | Operating steps in Chinese |
| `_can_listen.py` | Capture that receives only and transmits nothing. The log is `_can_listen_log.txt` in this directory |
| `check_candidate_repo.py` | After a candidate policy repository arrives, search its config, actions, normalization, cameras, and any Piper execution code |

Paths are relative to the directory that contains this code. Do not put someone's desktop absolute path into the docs or into new code. Server paths come from `team_config.json`.

`piper_sdk` and `piper_ros` are in the repository, but both use Linux `can0`. The control loop on Windows and macOS does not call them. CAN IDs and byte order were checked against the SDK; `piper_client.py` still sends the frames.

Running `python piper_client.py` queries the gripper limit and then performs one full open and close. The library call `read_state()` sends no motion command.

## piper_client.py

`PiperClient` is a context manager. On Windows, connect opens WinUSB with overlapped I/O. On macOS it uses libusb. Both set the adapter to 1 Mbps for a 48 MHz clock and start the controller.

`read_state(timeout=0.6)` sets `complete` to True only after all eight IDs below have arrived:

| Field | Source | Unit |
| --- | --- | --- |
| `joints_deg` | `0x2A5`–`0x2A7`, six values | degrees |
| `gripper_mm` | first 4 bytes of `0x2A8` | millimeters |
| `gripper_effort` | following uint16 in `0x2A8` | 0.001 N·m |
| `gripper_status` | status byte of `0x2A8` | raw value |
| `pose_mm_deg` | `x y z rx ry rz` from `0x2A2`–`0x2A4` | millimeters and degrees |
| `status` | `0x2A1` | see the protocol section |

Multi-byte integers are big-endian. Divide the raw value by 1000.

`gripper_max_mm()` sends query `0x477` with data `04 00 00 00 03 00 00 00`. If byte 1 of `0x47E` is 70 or 100, that value is used; otherwise the client falls back to 70. One arm answered 100. This query does not change the load setting.

`move_gripper(target_mm, effort=1000)` sends only `0x159`. The target is limited to 0–100 mm and the effort to 0–5000. The frame is repeated about every 10 ms until feedback is within 1 mm of the target, or until 4 seconds pass. It does not enable the joints. Byte 7 stays 0 so the current position is not written as the zero point.

One full gripper stroke has been verified, with the joint angles unchanged before and after. Read the actual angles from `read_state()` on site. Do not copy them into a shared config.

`exit_drag_teach()` sends `0x150` with data `00 00 02 00 00 00 00 00`. That ends a drag-teach recording. It is not an emergency stop and it does not write the zero point. `enable_arm()` sends `0x471` with `07 02` and six zeros, enabling every joint. `set_pose_mode(speed)` sends `0x151`: CAN control, MOVE P, and the speed percent, with the installation byte left at 0. `move_pose(...)` sends big-endian `0x152` (X, Y), `0x153` (Z, RX), and `0x154` (RY, RZ), in 0.001 mm and 0.001 degrees, and repeats the mode frame for the same interval. It can repeat one gripper frame in that same interval.

While `0x2A1` control mode is `0x02` (`TEACHING`) and the teach byte is `0x01`, end-pose commands are not followed. After leaving teach mode, one verified motion ended in `CAN_CTRL` with `motion=0`, and the pose read back was close to the commanded target. Bit 6 of byte 6 in low-speed feedback `0x261`–`0x266` means that joint driver is enabled.

Reads and writes use separate events. Each write waits for completion and drains the transmit echo; otherwise later writes stall. Close cancels pending I/O and releases the interface. It does not send the bus-stop control transfer, which hung the process. Putting the camera and CAN writes in one process exits with an access violation (`3221225477`), so `policy_link.py` hands the action array to a new `policy_step.py` process.

## Policy link

`policy_link.py` uses the same WebSocket as `openpi_client`: after connect, the server sends one msgpack metadata message; each later observation is one msgpack message, and each reply is one msgpack message. NumPy arrays travel in an `__ndarray__` map whose keys are the bytes `data`, `dtype`, and `shape`.

Server directory, account, host name, and model name live only in each person's `team_config.json`. If a field is missing, `team_config.py` names the keys to fill and does not reuse someone else's paths. Operators run `python team_config.py run`. It submits or reuses a job named `serve-policy` on the login node, waits until the log contains `listening on`, reads `NodeAddr` from `scontrol`, opens one local `ssh -L` to that address, then calls `policy_link.py once`. On Windows the forward stays in its own window. On macOS it runs in the background, and `down` stops it with `pkill`. `down` cancels that user's `serve-policy` jobs and stops the matching local `ssh -N`. `job` and `serve` still only print text for someone who needs to start the server by hand.

Order inside `once`:

1. Retry the connection to `ws://host:port` up to 50 times, with a 2-second handshake timeout. `python team_config.py once` and `python policy_link.py once` without `--port` both use `local_forward_port`. `python policy_link.py serve` still listens on `127.0.0.1:8765` by default, which is the local placeholder server.
2. Receive metadata within 30 seconds and print `server_connected`.
3. Call `read_state()`. If it is incomplete, raise and do not send.
4. `capture_color()` takes one RGB frame from the Orbbec Astra Pro color camera (USB `VID_2BC5` `PID_0501`) and pads it to 224×224. Other cameras on the same computer are not selected. `build_observation(state, image)` copies that same frame into all three image fields.
5. Send the observation and print `image_mean` and `waiting_for_inference`.
6. Wait up to 600 seconds. If the reply is a string, raise it as a `RuntimeError`. That is how an OpenPI server returns an exception.
7. Print `actions_shape`. One verified return was `(50, 12)`. Then execute the whole chunk in a separate process, and print `pose_sent` plus the state read afterward.
8. Return to step 3. `Ctrl+C` prints `stopped` and exits. The WebSocket stays open for the whole loop. The CAN client used to read state is closed before the motion process starts, so the two do not hold the adapter at the same time.

`python policy_link.py serve` is a local placeholder on `127.0.0.1:8765`. It only checks the observation shape currently in this code and returns a placeholder action. It is not a substitute for anyone's checkpoint.

## Observation contract

Image count, state dimension, prompt, and action meaning all come from the loaded model. They are not a fixed standard of this repository. When the model changes, edit `build_observation()`, and point `policy_config` and `checkpoint_dir` in `team_config.json` at that checkpoint.

The table below is what the current function sends. It is not everyone's input:

| Key | Value |
| --- | --- |
| `observation/image` | uint8, `(224, 224, 3)`, Orbbec Astra Pro color frame |
| `observation/wrist_image` | a copy of that same frame. There is no second camera |
| `observation/right_image` | a copy of that same frame. Some models do not need a third image; a missing key is reported by the server |
| `observation/state` | float32, shape `(16,)` |
| `prompt` | the string currently in the source. Change it for the task before a run. The language instruction is not typed on the server |

The order of these 16 numbers is hard-coded in `robocasa_state()`. If another model uses a different order or length, replace that function instead of reusing the table:

| Index | Content | How this client fills it |
| --- | --- | --- |
| 0–2 | end-effector position, meters | Piper `x y z` millimeters divided by 1000. This is an absolute pose, not the relative quantity used in training |
| 3–6 | end-effector quaternion, wxyz | `rx ry rz` degrees converted as intrinsic XYZ Euler angles |
| 7–9 | base position, meters | 0. This Piper has no mobile base |
| 10–13 | base quaternion, wxyz | `1 0 0 0` |
| 14–15 | two finger joint positions, meters | opening in millimeters divided by 2000, written into both fingers |

Do not change the msgpack packing. With only one Astra Pro, say that all three fields contain the same frame. The 16-value order has not been checked item by item against the training-state concatenation. Run the candidate-repository check before changing models. This control loop does not use a camera extrinsic, and no hand-eye calibration was done.

## Action contract

`action_waypoints()` in `policy_step.py` interprets the current 12-value output as follows. This matches one end-effector delta policy that was checked. It is not a universal format for every checkpoint. When the model changes, replace this function. Do not write the 12 values to `0x155` as six joint angles.

| Index | Meaning | How this client uses it |
| --- | --- | --- |
| 0–2 | end-effector translation, base frame, controller units near [-1, 1] | Multiply by 0.05 m, then by 1000 for millimeters, and add to the current `x y z`. +X is forward, +Y is left, +Z is up |
| 3–5 | axis-angle, same base frame, controller units near [-1, 1] | Multiply by 0.5 rad and left-multiply onto the current orientation. Euler angles match `piper_fk`: `R = Rz(rz) Ry(ry) Rx(rx)` |
| 6 | gripper | At or above 0.5 means close, target 0 mm; otherwise open to `gripper_max_mm()` |
| 7–10 | base and torso | This Piper has no such mechanisms, so these values are dropped |
| 11 | control mode | Not a length; dropped |

`execute_steps` in `team_config.json` keeps only the first N steps of each chunk (empty means all of them). `ee_delta_scale` scales translation and rotation (empty means 1). Before any frame is sent, `check_limits()` refuses a chunk that reaches farther than `max_chunk_shift_mm` from the start pose. Each kept step is sent at 0.05 s with speed 50%. After the chunk, `policy_link.py` requests the next one with a new image and state. Verified: after leaving teach mode, the arm followed one chunk's final pose, with `motion=0`.

## macOS and the simple server

Windows and macOS are both supported. `piper_client.py` chooses the transport by platform: `_WinUsbTransport` on Windows and `_LibUsbTransport` (pyusb, VID `1D50` / PID `606F`) on macOS. `open_transport()` sends the same gs_usb control transfers for both, and the 20-byte frames, echo draining, and every CAN ID are shared. `_can_listen.py` uses the libusb transport on macOS.

The libusb transport drains the IN endpoint from a background thread into a 256-frame queue (about 0.1 s of feedback; the oldest frame is dropped when full). Without that, feedback at about 2,700 frames/s exhausts the adapter's frame buffers and OUT writes time out with errno 60 after about a dozen frames. Do not go back to reading only between writes.

Setup on a Mac:

```bash
brew install libusb
python3 -m venv .venv
.venv/bin/pip install numpy msgpack 'websockets>=13' opencv-python pyusb pyobjc-framework-AVFoundation pandas pyarrow
```

OpenCV on macOS numbers cameras in `uniqueID` order and cannot select one by USB ID. `_avfoundation_devices()` rebuilds that order through pyobjc, and the Astra Pro is the device whose model ID contains `VendorID_11205 ProductID_1281` (0x2BC5 / 0x0501). A non-empty `camera_index` in `team_config.json` overrides it. `python policy_link.py cameras` lists indexes with names. `python policy_link.py preview [--index N]` opens a live window with the 224×224 model input inset; q closes it. Camera access must be allowed for the app that runs the command.

`policy_step.check_limits()` runs before any frame is sent. A waypoint farther than `max_chunk_shift_mm` (default 30) from the pose read before the chunk, or outside the absolute base-frame box `workspace_mm` (`{"x": [min, max], ...}`; an empty axis is skipped), raises and sends nothing.

`simple_policy_server.py` returns a scripted `(50, 12)` chunk with the action meaning in `action_waypoints()`: +X by 10 mm and back, and the gripper alternating between chunks. With `server_mode` set to `"simple"`, `team_config.py setup-remote` copies it, `policy_link.py`, and `piper_client.py` to `remote_code_dir` and builds a venv there; `run` then submits it without a GPU. `"openpi"` keeps the `serve_policy.py` launch. Off Windows, the local forward is `ssh -f -N -L` in the background, and `down` stops it with `pkill`.

## Joint-space actions

End-pose (MOVE P) targets from the folded rest pose are rejected by the arm's IK: status shows `TARGET_EXCEEDS_LIMIT` and only the gripper moves. A reply may therefore carry `"action_space": "joints"` with actions `(N, 7)`: six absolute joint angles in degrees and a gripper value (at or above 0.5 closes). `policy_link.py` passes the action space to `policy_step.py apply`; a reply without it is `ee_delta`.

`apply_joint_actions()` sends MOVE J (`0x151` byte 1 = `0x01`) and joints on `0x155` (J1, J2), `0x156` (J3, J4), `0x157` (J5, J6), big-endian int32 in 0.001 degrees, at 0.05 s per step, speed `joint_speed` (default 60%). Within each step `move_joints(start_deg=...)` interpolates from the previous target and resends about every 5–7 ms (about 140 Hz). MOVE J plans to each new target, so holding one target per 0.05 s step made J1 and J2 start and stop on every step; at 30% the arm also lagged the commanded rate. Before anything is sent, `check_joint_chunk()` refuses a step that moves any joint more than 3 degrees, or a chunk that moves a joint more than `max_chunk_joint_deg` (default 100) from the start. `move_joints()` refuses targets more than 2 degrees outside `JOINT_LIMITS_DEG` and clamps smaller excursions. The observation carries `observation/joints_deg` for scripted servers.

`simple_policy_server.py --mode demo` (the default) unfolds to `READY_DEG = [0, 45, -45, 0, 30, 0]` at half speed, then cycles a J1 wave (±30), a J5 nod with a J2/J3 reach, and a ±90 J6 twist with the gripper. Segment lengths scale so no joint exceeds 2.4 degrees per step. `--mode nudge` keeps the end-effector test.

In low-speed feedback `0x261`–`0x266`, the driver-enabled flag is bit 6 of byte 5 (0-based).

## Replay and server modes

Replay uses the drag-teach episodes from `episode_recorder.py` (see `数据采集.md`). `episode_arrays()` reads `j1_deg`–`j6_deg`, `gripper_mm`, and `t_robot` from `robot_feedback.parquet`, or the same arrays from an `.npz`. `setup-remote` writes that `.npz` with `write_replay_npz()` and uploads it as `episodes/<episode folder>.npz`, so the server venv needs no pandas. The recorder runs on Windows and macOS: `open_policy_camera()` picks the Astra Pro with DirectShow or AVFoundation, and `KeyReader` reads `s`/`f`/`d` through `msvcrt` or a cbreak terminal (restored on exit; macOS then sets `PENDIN`, which is harmless).

`simple_policy_server.py --mode replay --episode FILE` resamples onto 0.05 s steps with `np.interp`, marks the gripper closed below 5 mm (`dataset_export.GRIPPER_CLOSE_MM`), and splits any step above 2.4 degrees. Each reply is the next `--chunk` steps (default 50). When the arm is more than 2.4 degrees from the chunk's first row (first chunk, loop restart, or lag), a half-speed `keyframe_chunk()` bridge is prepended. At the end the reply is `{"actions": (0, 7), "done": True}` and `policy_link.py` prints `server_done` and returns; `--loop` restarts instead. Each connection replays from the start.

`team_config.py` takes `--server simple|replay|openpi` over `server_mode`, then applies `mode_overrides[mode]` (any keys, typically Slurm partition, memory, time). `replay` copies `replay_episode` to `remote_code_dir/episodes/` in `setup-remote`. `openpi` runs `openpi_python` (default `.venv/bin/python` in `openpi_dir`) on a GPU; `openpi_dir`, `openpi_python`, and `checkpoint_dir` may start with `~/`. The job carries `--comment=MODE`; `up` refuses to reuse a `serve-policy` job from another mode. When the job's address differs from the one saved in `piper_link_state.json`, `up` closes the old local forward and opens a new one.

`apply_actions()` keeps the first `execute_steps` rows (empty: all) and passes `ee_delta_scale` (empty: 1.0) to `action_waypoints()`, which multiplies translation and rotation. Limit checks raise `ChunkRefused`; `policy_step.py apply` then exits with status 3 and `policy_link.py` prints `chunk_refused` and observes again instead of stopping. The RoboCasa pi0.5 checkpoint answers in about 0.15 s after a first call of about 30 s; its unscaled chunks integrate to 0.3–1.5 m.

`policy_link.py home` uses the same `go_to()` path to fold to `HOME_DEG` (all zeros, the rest pose). `policy_link.py ready` and `once --ready` unfold to `READY_DEG` with joint targets before an `ee_delta` model, since end-pose targets from the folded pose are rejected.


The adapter is a candleLight, `VID_1D50` / `PID_606F`, CAN clock 48 MHz. On Windows it is WinUSB, interface GUID `{c15b4308-04d3-11e6-b3ea-6057189e6443}`. On macOS, libusb opens the same VID/PID.

These make `CreateFile` return error 5:

1. ArmRobotUA. It owns the device through `cando.dll` and exposes no external SDK or ROS port. After that program is closed, a non-administrator process can open the same interface.
2. `VMUSBArbService`. It was stopped during bring-up. Start it again by hand when a virtual machine needs USB.
3. A Python process from an earlier run that is stuck in the driver. Close that process. If the open still fails, unplug and replug the candleLight.

The open flags must include `FILE_FLAG_OVERLAPPED` (`0x40000000`). Without it, `WinUsb_Initialize` returns error 6. Reads and writes use `OVERLAPPED`; 997 means the I/O is still pending. The bulk pipes seen here are IN `0x81` and OUT `0x02`. On another adapter, confirm them with `WinUsb_QueryPipe`.

## Protocol

USB control transfers to the adapter and CAN frames to the Piper are different kinds of messages.

Control transfers: host-format request 0 with payload `0x0000BEEF`; bit-timing request 1; start request 2 with mode 1. For 48 MHz and 1 Mbps the timing is `prop_seg=1, phase_seg1=12, phase_seg2=2, sjw=1, brp=3`. Use `brp=5` only for 80 MHz.

A gs_usb frame is 20 bytes, with no hardware timestamp:

```text
0   echo_id  uint32 little-endian
4   can_id   uint32 little-endian
8   dlc
12  data     up to 8 bytes
```

`echo_id == 0xFFFFFFFF` is a frame received from the bus. `echo_id == 0` is the local transmit echo, not a reply from the arm. The standard ID is `can_id & 0x7FF`. The error-frame flag is `0x20000000`, and the client drops those frames.

While feedback is streaming, the approximate rates are:

| ID | Rate | Content |
| --- | --- | --- |
| `0x2A1` | about 190 Hz | control mode, arm status, motion mode, teach status, motion status, trajectory number, fault code as big-endian uint16 |
| `0x2A2`–`0x2A4` | about 190 Hz | end-effector XYZ and orientation |
| `0x2A5`–`0x2A7` | about 190 Hz | six joints |
| `0x2A8` | about 190 Hz | gripper stroke, effort, and status |
| `0x251`–`0x256` | about 95 Hz | high-speed drive feedback |
| `0x261`–`0x266` | about 19 Hz | low-speed drive feedback |

One captured `0x2A1` payload was `01 00 01 02 00 00 00 00`: CAN control, normal, MOVE_J, teach status 2, motion status 0, fault code 0. The enums are in `piper_sdk/.../arm_feedback_status.py`.

`0x710` appeared in an early short listen and was absent from a 25-second passive listen. It has no reliable meaning; do not answer it. `0x1C0`–`0x1C3` and `0x212` were also seen. The client does not interpret them.

The firmware query is `0x4AF` with data `01 00 00 00 00 00 00 00`. Concatenate received payloads and search for the ASCII `S-V`. Use the version string read on site. That reply only shows that the controller is on the bus. It does not mean periodic feedback is being sent.

The gripper command `0x159` is 8 bytes, big-endian: int32 stroke (0.001 mm), uint16 effort (0.001 N·m), uint8 enable code, uint8 set-zero. The set-zero byte must be `0x00`. `0xAE` writes the current position as the zero point and must not be sent. The enable code used here is `0x01`. Do not guess byte 3 of `0x477` in order to turn periodic feedback on; two comments in the SDK contradict each other.

## Do not switch to these entry points

| Entry point | Result |
| --- | --- |
| `C_PiperInterface_V2("can0")` | Linux socketcan only |
| `python-can` `gs_usb` / libusb | Access denied for this WinUSB device |
| `pyAgxArm` + `agx_cando` | Documented Windows route; not installed or verified on this machine |
| `piper_ros` | Linux `can0` |
| The ArmRobot UI, or the `QTcpSocket` inside it | Listens on no external port and owns the adapter |

## Check a candidate repository

When a candidate repository arrives, run this check before discussing how to map its output onto the Piper. Replace the path with the actual directory:

```powershell
python check_candidate_repo.py PATH_TO_REPO
```

The program searches for clues and prints file names and line numbers for five questions. Reading the cited files is still required. Do not change `build_observation()` or send joint commands until each item has a definite answer.

1. Config: one arm or two, and the dimensions of the input state and the output action.
2. Action: joint angles or end-effector pose, absolute or incremental, and the units.
3. Preprocessing and postprocessing: whether the training normalization statistics or processor files are included.
4. Cameras: which views are required, their field names, and their resolution.
5. Deployment code: whether it contains a Piper driver, a gripper mapping, and an execution rate.

If the action type, units, or camera fields are still uncertain, do not write an array found by this search to `0x155` or `0x159`. Change `action_waypoints()` first.

## Boundary

Verified on one Piper with a candleLight adapter: a passive capture collected `0x2A1`–`0x2A8`, a full gripper stroke left the joints unchanged, the WebSocket delivered an observation and received `(50, 12)`, and one end-effector chunk was followed after teach mode was left.

One Orbbec Astra Pro color camera is connected. Its color frame is in the observation and is copied into all three image fields. Continuous control is the outer loop: play the whole chunk, observe again, then infer again. Not done: separate wrist and right cameras, hand-eye calibration, an item-by-item check of the 16-value state against the training concatenation, joint-space `0x155` commands, and master/slave offsets. This loop does not consume a camera extrinsic. When a model uses a different action meaning, change `action_waypoints()` first.
