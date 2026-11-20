"""Record actual Isaac RGB renders; used only by the isolated native demo."""

import json
import subprocess
from pathlib import Path

import numpy as np


class DemoVideo:
    def __init__(self, runner, directory, title, fps=20, resolution=(800, 450)):
        import omni.replicator.core as rep
        import omni.usd
        from omni.kit.viewport.utility import get_active_viewport
        from pxr import Gf, UsdGeom
        from PIL import ImageFont

        self.rep = rep
        self.directory = Path(directory)
        self.title, self.fps = title, fps
        self.frames, self.next_time = 0, 0.0
        self.minimum = float("inf")
        stage = omni.usd.get_context().get_stage()
        camera = UsdGeom.Camera.Define(stage, "/World/CrowdDemoCamera")
        camera.CreateFocalLengthAttr().Set(35.0)
        camera.CreateHorizontalApertureAttr().Set(36.0)
        camera.CreateVerticalApertureAttr().Set(20.25)
        camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 1000.0))
        camera.MakeMatrixXform().Set(
            Gf.Matrix4d()
            .SetLookAt(Gf.Vec3d(0, 5, 12), Gf.Vec3d(0, 10, 0), Gf.Vec3d(0, 0, 1))
            .GetInverse()
        )
        viewport = get_active_viewport()
        if viewport is not None:
            viewport.camera_path = camera.GetPath()
        self.product = rep.create.render_product(str(camera.GetPath()), resolution)
        self.annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self.annotator.attach([self.product])
        font = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
        self.font = ImageFont.truetype(font, 17)
        width, height = resolution
        self.stderr = (self.directory / "video_encoder.log").open("w")
        self.encoder = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pixel_format",
                "rgb24",
                "-video_size",
                f"{width}x{height}",
                "-framerate",
                str(fps),
                "-i",
                "pipe:0",
                "-an",
                "-c:v",
                "libx264",
                "-threads",
                "2",
                "-preset",
                "fast",
                "-crf",
                "20",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(self.directory / "isaac_demo.mp4"),
            ],
            stdin=subprocess.PIPE,
            stderr=self.stderr,
        )
        for _ in range(3):
            runner.simulation_app.update()

    def capture(self, runner):
        from PIL import Image, ImageDraw

        t = float(runner.world.current_time)
        if not runner.play_started or t + 1e-6 < self.next_time:
            return
        pixels = self.annotator.get_data()
        if not isinstance(pixels, np.ndarray) or pixels.ndim != 3 or pixels.size == 0:
            return
        self.next_time += 1 / self.fps
        image = Image.fromarray(np.ascontiguousarray(pixels[:, :, :3]))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, image.width, 75), fill=(17, 24, 39))
        draw.text((12, 8), self.title, font=self.font, fill="white")
        backend = runner.people[0]._crowd_runtime
        status = ", ".join(f"P{i}: {s}" for i, s in enumerate(backend.social.states))
        draw.text((12, 32), f"Isaac Sim | t={t:.1f}s | {status}", font=self.font, fill="white")
        diag = backend.social.diagnostics()
        distance = diag["minimum_pedestrian_separation_m"]
        if len(runner.people) == 1:
            distance = diag["minimum_robot_separation_m"]
        if distance is not None:
            draw.text(
                (12, 54),
                f"Minimum observed center distance: {distance:.2f} m",
                font=self.font,
                fill=(125, 211, 252),
            )
        self.encoder.stdin.write(np.ascontiguousarray(image).tobytes())
        self.frames += 1
        if not (self.directory / "overview.png").exists() and t >= 1.0:
            image.save(self.directory / "overview.png")
        if distance is not None and distance < self.minimum:
            self.minimum = distance
            image.save(self.directory / "encounter.png")
        if any(s in ("yielding", "stepping_aside") for s in backend.social.states):
            if not (self.directory / "yield.png").exists():
                image.save(self.directory / "yield.png")
        image.save(self.directory / "last_frame.png") if self.frames % self.fps == 0 else None

    def close(self):
        self.encoder.stdin.close()
        returncode = self.encoder.wait(timeout=30)
        self.stderr.close()
        if returncode:
            raise RuntimeError("Video encoder failed; see video_encoder.log")
        (self.directory / "video.json").write_text(
            json.dumps(
                {
                    "source": "Isaac Sim RGB render",
                    "frames": self.frames,
                    "fps": self.fps,
                    "duration_sec": self.frames / self.fps,
                },
                indent=2,
            )
        )
        self.annotator.detach([self.product.path])
        self.product.destroy()
