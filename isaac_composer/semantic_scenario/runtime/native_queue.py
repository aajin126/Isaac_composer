"""Reuse installed People QueueCmd/Dequeue while ORCA owns locomotion."""

import math

from ..schema import PlanError
from .compiler import native_path


class PoseIntent:
    """Native commands can read pose but cannot bypass the final animation writer."""

    def __init__(self, person):
        self.person = person

    def get_world_transform(self, position, rotation):
        p = self.person.state.position
        position.x, position.y, position.z = map(float, p)
        rotation.x, rotation.y, rotation.z, rotation.w = 0.0, 0.0, 0.0, 1.0

    def set_variable(self, *args):
        # Action/Walk/PathPoints are written once by pedestrians.guard_crowd.
        pass


class QueueNavigation:
    def __init__(self, scene, person, speed, arrival_radius):
        self.scene, self.person, self.speed = scene, person, speed
        self.radius = arrival_radius
        self.target = None
        self.measured_speed = 0.0

    def generate_path(self, points, rotation=None):
        self.target = list(map(float, points[-1]))
        path = native_path(self.scene, self.person.state.position, self.target)
        self.person._current_point_index = 0
        self.person.update_target_position(path, walk_speed=self.speed)

    def generate_goto_path(self, points):
        self.generate_path([self.person.state.position, list(map(float, points[:3]))])

    def check_proximity_to_point(self, point, unused_threshold):
        # Native default is 1.5 m. Occupy a slot only after actual arrival.
        return math.dist(self.person.state.position[:2], list(point)[:2]) <= self.radius

    def destination_reached(self):
        return (
            self.target is not None
            and self.check_proximity_to_point(self.target, None)
            and self.measured_speed <= 0.15
        )

    def set_path_points(self, *args):
        pass

    def set_path_target_rot(self, *args):
        pass

    def clean_path_targets(self):
        pass


class NativeQueues:
    def __init__(self, scene, people, definitions, speeds, arrival_radius=0.3):
        import carb
        from omni.anim.people.scripts.global_queue_manager import GlobalQueueManager
        from omni.anim.people.scripts.commands.queue import QueueCmd
        from omni.anim.people.scripts.commands.dequeue import Dequeue

        # Only the movement adapter changes; admission and advancement are the
        # installed QueueCmd implementation, not a second queue algorithm.
        class OrcaMovement:
            def walk(self, dt):
                reached = self.navigation_manager.destination_reached()
                self.navigation_manager.person._target_speed = (
                    0.0 if reached else self.navigation_manager.speed
                )
                return reached

            def exit_command(self):
                return True

        class OrcaQueue(OrcaMovement, QueueCmd):
            pass

        class OrcaDequeue(OrcaMovement, Dequeue):
            pass

        self.manager = GlobalQueueManager.get_instance()
        self.names = {q["queue_id"]: "semantic_" + q["queue_id"] for q in definitions}
        for definition in definitions:
            queue = self.manager.create_queue(self.names[definition["queue_id"]])
            for index, point in enumerate(definition["positions"]):
                queue.create_spot(index, carb.Float3(*point), carb.Float4(0, 0, 0, 1))
        self.people, self.scene, self.speeds = people, scene, speeds
        self.radius = arrival_radius
        self.command_types = {"Queue": OrcaQueue, "Dequeue": OrcaDequeue}
        self.commands = {}

    def begin(self, aid, action, queue_id, target=None):
        person = self.people[aid]
        name = person._stage_prefix
        native_name = self.names[queue_id]
        if (
            action == "Dequeue"
            and self.manager.get_queue(native_name).get_spot(0).get_occupier() != name
        ):
            raise PlanError("Only the actor at the front may leave a queue")
        command = [action, native_name]
        if target is not None:
            command += [str(float(x)) for x in target] + ["_"]
        nav = QueueNavigation(self.scene, person, self.speeds[aid], self.radius)
        self.commands[aid] = self.command_types[action](
            queue_manager=self.manager,
            character=PoseIntent(person),
            command=command,
            navigation_manager=nav,
            character_name=name,
            command_id=f"semantic_{aid}_{action}",
            update_metadata_callback_fn=lambda **kwargs: None,
        )

    def step(self, aid, dt, measured_speed):
        command = self.commands[aid]
        nav = command.navigation_manager
        nav.measured_speed = float(measured_speed)
        spot = getattr(command, "current_spot", None)
        if spot is not None and not command.walking:
            if not nav.check_proximity_to_point(spot.get_translation(), None):
                nav.generate_path([nav.person.state.position, spot.get_translation()])
                command.walking = True
        done = bool(command.execute(dt))
        waiting = not getattr(command, "walking", True)
        if done:
            self.commands.pop(aid)
            nav.person._target_speed = 0.0
        return done, waiting

    def close(self):
        for native_name in self.names.values():
            queue = self.manager.get_queue(native_name)
            for person in self.people:
                queue.free_queue_spot(person._stage_prefix)
        self.commands.clear()
