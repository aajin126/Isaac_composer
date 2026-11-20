"""Advance walking at each character's rate and publish the pose to the scene."""


class CharacterClock:
    """Own the graph clock so automatic and explicit time cannot accumulate.

    Install before enabling omni.anim.graph.core. Initialize after world reset,
    and advance with elapsed simulation time, never wall time or physics_dt.
    """

    def __init__(self, settings):
        self.settings = settings
        self.previous_manual = bool(settings.get("/animGraph/manualUpdate"))
        self.interface = None
        settings.set_bool("/animGraph/manualUpdate", True)

    def start(self, interface, stage_id):
        if not interface.initialize(stage_id):
            raise RuntimeError("Could not initialize the manual pedestrian animation clock")
        self.interface = interface

    def advance(self, dt, people):
        if self.interface is None or dt <= 0:
            return
        for person in people:
            if person.character_graph:
                person.character_graph.update(dt * getattr(person, "_animation_playback_rate", 1.0))
        # A positive manager tick publishes transforms/skin poses to Fabric.
        # Characters explicitly updated this frame retain their custom dt;
        # a zero-dt manager tick does not publish their internal state.
        if not self.interface.update(dt):
            raise RuntimeError("Could not update the manual pedestrian animation clock")
        # The rendered root pose is now current. Move the kinematic collision
        # body to the same position before the next physics step.
        for person in people:
            if getattr(person, "_collision_capsule", None) is not None:
                person.update_state(0.0)

    def close(self):
        if self.interface is not None:
            self.interface.shutdown()
            self.interface = None
        self.settings.set_bool("/animGraph/manualUpdate", self.previous_manual)
