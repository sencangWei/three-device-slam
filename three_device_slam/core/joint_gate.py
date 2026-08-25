from enum import Enum


class GateState(Enum):
    COLD = "cold"
    WARMING = "warming"
    JOINT_READY = "joint_ready"
    RECORDING = "recording"
    BLOCKED = "blocked"


class JointWarmupGate:
    def __init__(self, device_ids, warmup_ns, continuous_healthy_ns, timeout_ns):
        self._device_ids = tuple(device_ids)
        self._warmup_ns = warmup_ns
        self._continuous_healthy_ns = continuous_healthy_ns
        self._timeout_ns = timeout_ns
        self._latest = {}
        self._last_timestamp_ns = {}
        self._first_timestamp_ns = {}
        self._healthy_run_started_ns = {}
        self._recording_requested_ns = None
        self.state = GateState.COLD
        self.task_start_ego_frame_ns = None

    def observe(self, heartbeat):
        if heartbeat.device_id not in self._device_ids:
            raise ValueError(f"unknown device: {heartbeat.device_id}")
        previous_ns = self._last_timestamp_ns.get(heartbeat.device_id)
        if previous_ns is not None and heartbeat.at_ns < previous_ns:
            raise ValueError(f"timestamp regression for {heartbeat.device_id}")

        self._last_timestamp_ns[heartbeat.device_id] = heartbeat.at_ns
        self._latest[heartbeat.device_id] = heartbeat
        if heartbeat.device_id not in self._first_timestamp_ns:
            self._first_timestamp_ns[heartbeat.device_id] = heartbeat.at_ns
            if self.state is GateState.COLD:
                self.state = GateState.WARMING

        if not heartbeat.healthy:
            self._healthy_run_started_ns[heartbeat.device_id] = None
            if self.state is GateState.JOINT_READY:
                self.state = GateState.WARMING
        elif self._healthy_run_started_ns.get(heartbeat.device_id) is None:
            self._healthy_run_started_ns[heartbeat.device_id] = heartbeat.at_ns

        self._advance(heartbeat.at_ns)

    def tick(self, now_ns):
        self._advance(now_ns)

    def request_recording(self, request_ns):
        if self.state is not GateState.JOINT_READY:
            raise RuntimeError("recording requires JOINT_READY state")
        self._recording_requested_ns = request_ns
        self.state = GateState.RECORDING

    def observe_ego_frame(self, frame_ns, healthy):
        if (
            self.state is GateState.RECORDING
            and self.task_start_ego_frame_ns is None
            and healthy
            and frame_ns >= self._recording_requested_ns
        ):
            self.task_start_ego_frame_ns = frame_ns
            return frame_ns
        return None

    def _advance(self, now_ns):
        if self.state in (GateState.BLOCKED, GateState.RECORDING, GateState.JOINT_READY):
            return
        if not self._first_timestamp_ns:
            return
        timeout_anchor_ns = min(self._first_timestamp_ns.values())
        if now_ns - timeout_anchor_ns >= self._timeout_ns:
            self.state = GateState.BLOCKED
            return
        if len(self._first_timestamp_ns) != len(self._device_ids):
            return
        if not all(heartbeat.healthy for heartbeat in self._latest.values()):
            return
        warmup_anchor_ns = max(self._first_timestamp_ns.values())
        stable_anchor_ns = max(self._healthy_run_started_ns.values())
        if (
            now_ns - warmup_anchor_ns >= self._warmup_ns
            and now_ns - stable_anchor_ns >= self._continuous_healthy_ns
        ):
            self.state = GateState.JOINT_READY
