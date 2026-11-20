"""ROS publishing adapter; C++ OmniGraph avoids Python-version ABI coupling."""


class ROSStatePublisher:
    def __init__(self, topic):
        import omni.usd
        import omni.graph.core as og

        self.og = og
        self.stage = omni.usd.get_context().get_stage()
        self.connected = False
        self.graph = "/World/PedestrianStateGraph"
        keys = self.og.Controller.Keys
        self.og.Controller.edit(
            {"graph_path": self.graph, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("Tick", "omni.graph.action.OnPlaybackTick"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("Packet", "omni.graph.nodes.ConstantToken"),
                    ("Publisher", "isaacsim.ros2.bridge.ROS2Publisher"),
                ],
                keys.SET_VALUES: [
                    ("Context.inputs:useDomainIDEnvVar", True),
                    ("Packet.inputs:value", "{}"),
                    ("Publisher.inputs:messageName", "String"),
                    ("Publisher.inputs:messagePackage", "std_msgs"),
                    ("Publisher.inputs:messageSubfolder", "msg"),
                    ("Publisher.inputs:topicName", topic),
                ],
                keys.CONNECT: [
                    ("Tick.outputs:tick", "Publisher.inputs:execIn"),
                    ("Context.outputs:context", "Publisher.inputs:context"),
                ],
            },
        )

    def publish(self, data):
        if not self.connected:
            attribute = self.og.Controller.attribute(f"{self.graph}/Publisher.inputs:data")
            if attribute.is_valid():
                self.og.Controller.connect(f"{self.graph}/Packet.inputs:value", attribute)
                self.connected = True
        self.og.Controller.set(
            self.og.Controller.attribute(f"{self.graph}/Packet.inputs:value"), data
        )

    def close(self):
        if self.stage.GetPrimAtPath(self.graph).IsValid():
            self.stage.RemovePrim(self.graph)
