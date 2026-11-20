"""Timestamped crowd ground truth and a stable recording schema."""

import json
from pathlib import Path
from ..state import PEDESTRIAN_FIELDS, STATE_CODES
from ..config import MU_DEFINITION
from ..behavior.stiffness import SocialStiffness
from ..behavior.responsibility import responsibility_settings


class StateRecorder:
    def __init__(self, config):
        self.config = config
        self.stream = None
        self.last_flush = 0
        directory = (
            self.config.get("state_output_dir")
            if self.config["social_navigation"].get("record_states_enabled", True)
            else None
        )
        if directory:
            path = Path(directory)
            path.mkdir(parents=True, exist_ok=True)
            self.stream = (path / "crowd_states.jsonl").open("w")
            (path / "crowd_state_schema.json").write_text(
                json.dumps(
                    {
                        "frame_id": "world",
                        "pedestrian_fields": PEDESTRIAN_FIELDS,
                        "state_codes": STATE_CODES,
                        "density_includes_self": True,
                        "density_is_telemetry_only": True,
                        "density_radius_m": config["social_navigation"]["density_radius_m"],
                        "yield_trigger": "persistent_lack_of_useful_orca_velocity",
                        "progress": {
                            key: config["social_navigation"][key]
                            for key in (
                                "progress_hold_s",
                                "progress_min_mps",
                                "progress_min_fraction",
                            )
                        },
                        "area_definition": "NavMesh effective center-walkable area",
                        "control_rate_hz": 50,
                        "mu_definition": MU_DEFINITION,
                        "social_stiffness": SocialStiffness(
                            config["people"], config["social_navigation"]
                        ).diagnostics(),
                        "deadlock_recovery_enabled": config["social_navigation"][
                            "yielding_enabled"
                        ],
                        "robot_avoidance_responsibility": 1.0,
                        "directional_responsibility": responsibility_settings(
                            config["social_navigation"]
                        ),
                        "names": [
                            p.get("name", f"Character_{i}")
                            for i, p in enumerate(self.config["people"])
                        ],
                    },
                    indent=2,
                )
            )

    def write(self, packet, serialized=None):
        if self.stream:
            data = (
                serialized
                if serialized is not None
                else json.dumps(packet, allow_nan=False, separators=(",", ":"))
            )
            self.stream.write(data + "\n")
            stamp = packet["stamp_ns"] / 1e9
            if stamp - self.last_flush >= 1:
                self.stream.flush()
                self.last_flush = stamp

    def close(self):
        if self.stream:
            self.stream.close()
            self.stream = None
