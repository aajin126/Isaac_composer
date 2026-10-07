"""Built-in sensor providers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from isaac_composer.config import load_sensor_preset
from isaac_composer.models import RobotContext, SensorHandle
from isaac_composer.registry import sensor_providers
from isaac_composer.stage import set_xform_pose


class _ProviderBase:
    def __init__(
        self,
        *,
        package_root: str | Path,
        config_root: str | Path | None = None,
    ) -> None:
        self.package_root = Path(package_root)
        self.config_root = (
            Path(config_root)
            if config_root is not None
            else self.package_root / "config"
        )


@sensor_providers.register("embedded")
class EmbeddedSensorProvider(_ProviderBase):
    """Represent sensors and publishers already authored in a robot USD."""

    def setup(
        self,
        robot: RobotContext,
        config: dict[str, Any],
    ) -> list[SensorHandle]:
        del robot, config
        return []

    def detach(self, sensor: SensorHandle) -> None:
        del sensor

    def shutdown(self) -> None:
        return None


@sensor_providers.register("generated")
class GeneratedSensorProvider(_ProviderBase):
    """Create configured RTX LiDAR prims and apply reusable profiles."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.handles: list[SensorHandle] = []

    @staticmethod
    def _parent_path(
        robot: RobotContext,
        mount: dict[str, Any],
        handles: dict[str, SensorHandle],
    ) -> str:
        parent = mount["parent"]
        kind, separator, alias = parent.partition(":")
        if not separator:
            raise ValueError(f"Sensor mount parent must contain a kind: {parent}")
        if kind == "link":
            return robot.require_link(alias).prim_path
        if kind == "sensor_mount":
            try:
                return handles[alias].mount_prim_path
            except KeyError as error:
                raise KeyError(
                    f"Sensor mount refers to unknown earlier sensor {alias!r}"
                ) from error
        raise ValueError(f"Unsupported sensor mount parent kind: {kind}")

    @staticmethod
    def _create_rtx_lidar(parent_path: str, prim_name: str, base_config: str):
        import omni.kit.commands
        from pxr import Gf

        success, sensor_prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=prim_name,
            parent=parent_path,
            config=base_config,
            translation=Gf.Vec3d(0.0, 0.0, 0.0),
            orientation=Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0)),
            visibility=True,
        )
        if not success:
            raise RuntimeError(f"Failed to create RTX LiDAR: {parent_path}/{prim_name}")
        return sensor_prim

    def _apply_legacy_lidar_profile(
        self,
        sensor_prim_path: str,
        implementation: dict[str, Any],
    ) -> None:
        import omni.usd
        from pxr import Vt

        source = Path(implementation["source_json"])
        if not source.is_absolute():
            source = self.package_root / source
        source = source.resolve()
        if self.package_root.resolve() not in source.parents:
            raise ValueError(f"Sensor profile escapes package root: {source}")
        with source.open("r", encoding="utf-8") as stream:
            profile = json.load(stream)["profile"]

        prim = omni.usd.get_context().get_stage().GetPrimAtPath(sensor_prim_path)
        scalar_attributes = {
            "omni:sensor:Core:nearRangeM": "nearRangeM",
            "omni:sensor:Core:farRangeM": "farRangeM",
            "omni:sensor:Core:rangeResolutionM": "rangeResolutionM",
            "omni:sensor:Core:rangeAccuracyM": "rangeAccuracyM",
            "omni:sensor:Core:avgPowerW": "avgPowerW",
            "omni:sensor:Core:minReflectance": "minReflectance",
            "omni:sensor:Core:minReflectionRangeM": "minReflectanceRange",
            "omni:sensor:Core:waveLengthNm": "wavelengthNm",
            "omni:sensor:Core:azimuthErrorMean": "azimuthErrorMean",
            "omni:sensor:Core:azimuthErrorStd": "azimuthErrorStd",
            "omni:sensor:Core:elevationErrorMean": "elevationErrorMean",
            "omni:sensor:Core:elevationErrorStd": "elevationErrorStd",
            "omni:sensor:Core:scanRateBaseHz": "scanRateBaseHz",
            "omni:sensor:Core:reportRateBaseHz": "reportRateBaseHz",
        }
        for attribute_name, profile_name in scalar_attributes.items():
            prim.GetAttribute(attribute_name).Set(float(profile[profile_name]))
        prim.GetAttribute("omni:sensor:Core:numberOfEmitters").Set(
            int(profile["numberOfEmitters"])
        )
        emitter = profile["emitterStates"][0]
        prim.GetAttribute("omni:sensor:Core:emitterState:s001:azimuthDeg").Set(
            Vt.FloatArray([float(value) for value in emitter["azimuthDeg"]])
        )
        prim.GetAttribute("omni:sensor:Core:emitterState:s001:elevationDeg").Set(
            Vt.FloatArray([float(value) for value in emitter["elevationDeg"]])
        )

    def setup(
        self,
        robot: RobotContext,
        config: dict[str, Any],
    ) -> list[SensorHandle]:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        handles_by_name: dict[str, SensorHandle] = {}
        for item in config.get("items", []):
            if not item.get("enabled", True):
                continue
            if item["type"] != "rtx_lidar":
                raise ValueError(f"Unsupported generated sensor type: {item['type']}")

            preset = load_sensor_preset(item["preset"], self.config_root)
            implementation = preset["implementation"]
            if implementation["provider"] != "isaac_sim_5_legacy_profile":
                raise ValueError(
                    "Unsupported RTX LiDAR profile provider: "
                    f"{implementation['provider']}"
                )

            mount = item["mount"]
            parent_path = self._parent_path(robot, mount, handles_by_name)
            mount_name = mount.get("prim_name", f"{item['name']}_link")
            sensor_prim_name = item.get("prim_name", item["name"])
            mount_path = f"{parent_path}/{mount_name}"
            sensor_path = f"{mount_path}/{sensor_prim_name}"
            stage.DefinePrim(mount_path, "Xform")
            set_xform_pose(
                stage,
                mount_path,
                mount["translation"],
                mount.get("orientation_quat_wxyz", [1.0, 0.0, 0.0, 0.0]),
            )
            self._create_rtx_lidar(
                mount_path,
                sensor_prim_name,
                implementation.get("base_config", "Example_Rotary"),
            )
            self._apply_legacy_lidar_profile(sensor_path, implementation)

            ros = item.get("ros", {})
            handle = SensorHandle(
                name=item["name"],
                sensor_type=item["type"],
                mount_prim_path=mount_path,
                sensor_prim_path=sensor_path,
                frame_id=ros.get("frame_id", mount_name),
                topic=ros.get("topic"),
                metadata={
                    "ros": dict(ros),
                    "parent_frame_id": self._parent_frame_id(
                        robot,
                        mount,
                        handles_by_name,
                    ),
                    "translation": list(mount["translation"]),
                    "orientation_quat_wxyz": list(
                        mount.get(
                            "orientation_quat_wxyz",
                            [1.0, 0.0, 0.0, 0.0],
                        )
                    ),
                },
            )
            handles_by_name[handle.name] = handle
            self.handles.append(handle)
        return list(self.handles)

    @staticmethod
    def _parent_frame_id(
        robot: RobotContext,
        mount: dict[str, Any],
        handles: dict[str, SensorHandle],
    ) -> str:
        kind, _, alias = mount["parent"].partition(":")
        if kind == "link":
            return robot.require_link(alias).frame_id
        return handles[alias].frame_id

    def detach(self, sensor: SensorHandle) -> None:
        import omni.usd

        render_product = sensor.render_product
        if render_product is not None and hasattr(render_product, "destroy"):
            render_product.destroy()
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(sensor.mount_prim_path)
        if prim and prim.IsValid():
            stage.RemovePrim(sensor.mount_prim_path)

    def shutdown(self) -> None:
        for sensor in reversed(self.handles):
            self.detach(sensor)
        self.handles.clear()
