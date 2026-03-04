# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

EWolf-BFMC2026 is the autonomous driving stack for Team EWolf competing in the Bosch Future Mobility Challenge (BFMC) 2026. The system runs on a Raspberry Pi 5 controlling a 1/10 scale vehicle. It can also run in simulation via ROS/Gazebo.
Style: PEP8 for Python, Google for C++.

## Commands

### Setup (first time or after dependency changes)
```bash
chmod +x setup.sh && ./setup.sh
```
This installs system APT packages, Node.js 20 + Angular CLI, and all Python deps globally via `sudo pip3`.

### Run the system
```bash
python3 main.py
```

### Simulation mode
Set `IS_SIMULATION = True` in `main.py` before running. Requires a running ROS/Gazebo environment with the `/automobile/image_raw` and `/automobile/command` topics.

### Add a new component
```bash
python3 newComponent.py
```
Enter the package name and category when prompted. This scaffolds a `process<Name>.py` + `thread<Name>.py` and auto-injects the import and instantiation into `main.py` between the designated comment markers.

### Dashboard (Angular frontend)
```bash
cd src/dashboard/frontend
npm start
```
The dashboard communicates with the backend via Flask-SocketIO on the Python side.

## Architecture

### Process/Thread Model

The entire system uses Python's `multiprocessing`. Each subsystem is a **WorkerProcess** (`src/templates/workerprocess.py`) — a `multiprocessing.Process` subclass — that owns one or more **ThreadWithStop** (`src/templates/threadwithstop.py`) worker threads.

The pattern is:
```
main.py
  └── processX (WorkerProcess / multiprocessing.Process)
        └── threadX (ThreadWithStop / threading.Thread)
        └── threadY ...
```

### Message Bus (Gateway Pattern)

Inter-process communication uses a shared `queueList` dictionary with four priority queues: `Critical`, `Warning`, `General`, and `Config`.

`processGateway` (`src/gateway/`) acts as a pub/sub router. Threads subscribe by sending a config message to the `Config` queue, and messages are delivered via `multiprocessing.Pipe`. Every message type is defined as an `Enum` in `src/utils/messages/allMessages.py` with fields: `Queue`, `Owner`, `msgID`, and `msgType`.

Sending a message:
```python
sender = messageHandlerSender(self.queuesList, SomeMessage)
sender.send(value)
```

Receiving a message (subscribe at init time):
```python
sub = messageHandlerSubscriber(self.queuesList, SomeMessage, "lastOnly", True)
value = sub.receive()  # Returns None if no new data
```

### Autonomous Driving Pipeline (AUTO mode)

Data flows through these layers in order:

1. **Sensing** — `processCamera` (`src/hardware/camera/`) runs `threadCamera` (Picamera2 capture) + `threadLane` (BEV lane detection → `LaneData`) + `threadSigns` (YOLO sign detection → `SignDetection`). `threadCamera` stores raw frames in a shared in-process dict (`shared_container['frame']`) that `threadLane` and `threadSigns` read without copying.

2. **Sensing** — `processLidar` (`src/hardware/Lidar/`) runs `threadReader` (raw RPLidar scan → `shared_container['last_scan']`) + `threadDetector` (filters 30° front arc, reports `LidarObstacle`).

3. **Decision** — `processControl` (`src/control/Control/`) runs `threadFSM` (Finite State Machine) + `threadControl` (actuator execution). `threadFSM` subscribes to `LaneData`, `LidarObstacle`, and `SignDetection`, runs at 100 Hz, and publishes a `ControlAction` dict. `threadControl` reads `ControlAction` and implements the **Stanley Controller** to compute steering (deci-degrees) and speed (mm/s) strings sent to the NUCLEO board via `SpeedMotor` / `SteerMotor` messages.

4. **Actuation** — `processSerialHandler` (`src/hardware/serialhandler/`) forwards motor commands over UART to the STM32 NUCLEO board, and reads back `BatteryLvl`, `ImuData`, `CurrentSpeed`, etc.

5. **Simulation bridge** — When `IS_SIMULATION = True`, `processsimRos` (`src/hardware/simRos/`) replaces the camera and serial handler, bridging the internal queue bus to ROS topics.

### State Machine

`StateMachine` (`src/statemachine/stateMachine.py`) is a multiprocessing-safe singleton (one instance per OS process). It manages system modes defined in `SystemMode` (`src/statemachine/systemMode.py`):

- `DEFAULT` — Camera on, everything else off.
- `AUTO` — Full pipeline: Lidar + Control enabled.
- `MANUAL` — Camera only; no autonomous control.
- `LEGACY` — Lidar + Semaphores + TrafficCom, but no new Control stack.
- `STOP` — Everything disabled.

Mode transitions are validated by `TransitionTable` (`src/statemachine/transitionTable.py`). When a transition occurs, a `StateChange` message (string name of the new mode) is broadcast on the `Critical` queue. Every process and thread that cares about mode changes subscribes to `StateChange` and reads `SystemMode[message].value` to get its per-process configuration.

In `main.py`, the main loop listens for `StateChange` and dynamically starts/stops `processLidar` and `processControl` based on whether the new mode enables them.

### FSM Behavior States (`src/control/Control/threads/allStates.py`)

`threadFSM` transitions between: `IDLE → LANE_FOLLOWING → DECELERATING → STOP_ACTION → INTERSECTION`, with side-states `HIGHWAY_DRIVING`, `EMERGENCY_BRAKE`, `PARKING_MANEUVER`, and `ROUNDABOUT`. Obstacle proximity is classified into `ObstacleZone` (CLEAR / WARNING / DANGER) based on Lidar distance (mm). Speed limits are in `SpeedLimit` (m/s).

### Stanley Controller (`threadControl`)

Parameters in `threadControl.__init__`:
- `k = 1.25` — cross-track gain
- `ks = 0.5` — softening constant
- `kd = 0.25` — derivative damping gain

Commands are scaled before serial dispatch: speed in mm/s (m/s × 1000), steering in deci-degrees (deg × 10), both clipped to ±500 mm/s and ±250 deci-degrees.

## Hardware vs. Simulation Toggle

In `main.py`, the single flag `IS_SIMULATION = False` controls whether hardware processes (`processCamera`, `processSerialHandler`) or the ROS bridge (`processsimRos`) are loaded. The Perception threads (`threadLane`, `threadSigns`) live inside `processCamera` in both cases — the simulation bridge feeds frames into the same `shared_container`.

## Key Files Reference

| File | Purpose |
|---|---|
| `main.py` | Entry point; wires all processes and starts the system |
| `newComponent.py` | Scaffold generator for new process/thread pairs |
| `src/utils/messages/allMessages.py` | All message type definitions (Enums) |
| `src/statemachine/systemMode.py` | Per-mode process enable/disable configuration |
| `src/statemachine/transitionTable.py` | Valid mode transitions |
| `src/control/Control/threads/allStates.py` | BehaviorState, SignType, ObstacleZone, SpeedLimit enums |
| `src/templates/workerprocess.py` | Base class for all processes |
| `src/templates/threadwithstop.py` | Base class for all threads |
