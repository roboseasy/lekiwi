"""Observe Nav2 lifecycle activation before the customer sends a goal."""

import time


NAV2_LIFECYCLE_NODES = (
    'map_server', 'amcl', 'controller_server', 'smoother_server',
    'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower',
    'velocity_smoother', 'collision_monitor',
)


class LifecycleReadiness:
    def __init__(self, node, *, service_type=None, clock=time.monotonic):
        if service_type is None:
            from lifecycle_msgs.srv import GetState
            service_type = GetState
        self.service_type = service_type
        self.clock = clock
        self.clients = {name: node.create_client(service_type, f'/{name}/get_state')
                        for name in NAV2_LIFECYCLE_NODES}
        self.pending = {}
        self.states = {}
        self.next_poll = 0.0

    def ready(self):
        for name, future in list(self.pending.items()):
            if future.done():
                response = future.result()
                self.states[name] = None if response is None else response.current_state.id
                del self.pending[name]
        now = self.clock()
        if now >= self.next_poll:
            self.next_poll = now + 0.25
            for name, client in self.clients.items():
                if name not in self.pending:
                    self.states.pop(name, None)
                    if client.service_is_ready():
                        self.pending[name] = client.call_async(self.service_type.Request())
        # lifecycle_msgs/State.PRIMARY_STATE_ACTIVE is 3. Service discovery
        # alone does not imply that an action server will accept a goal.
        return len(self.states) == len(self.clients) and all(
            state == 3 for state in self.states.values())
