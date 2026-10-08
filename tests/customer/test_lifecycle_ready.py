"""Regression for discovered but inactive Nav2 action servers."""

from concurrent.futures import Future
from types import SimpleNamespace
import unittest

from lifecycle_ready import LifecycleReadiness, NAV2_LIFECYCLE_NODES


class Client:
    available = True

    def service_is_ready(self):
        return self.available

    def call_async(self, _request):
        self.future = Future()
        return self.future


class Node:
    def __init__(self):
        self.clients = {}

    def create_client(self, _service, path):
        self.clients[path.split('/')[1]] = Client()
        return self.clients[path.split('/')[1]]


class LifecycleReadinessTest(unittest.TestCase):
    def setUp(self):
        self.now = 1.0
        self.node = Node()
        self.observer = LifecycleReadiness(
            self.node, service_type=SimpleNamespace(Request=SimpleNamespace),
            clock=lambda: self.now)

    def respond(self, name, state):
        self.node.clients[name].future.set_result(
            SimpleNamespace(current_state=SimpleNamespace(id=state)))

    def test_discovery_and_inactive_navigator_do_not_allow_a_goal(self):
        self.assertFalse(self.observer.ready())
        for name in NAV2_LIFECYCLE_NODES:
            self.respond(name, 2 if name == 'bt_navigator' else 3)
        self.assertFalse(self.observer.ready())
        self.now += 0.3
        self.assertFalse(self.observer.ready())
        for name in NAV2_LIFECYCLE_NODES:
            self.respond(name, 3)
        self.assertTrue(self.observer.ready())

    def test_pending_collision_monitor_blocks_other_active_nodes(self):
        self.assertFalse(self.observer.ready())
        for name in NAV2_LIFECYCLE_NODES:
            if name != 'collision_monitor':
                self.respond(name, 3)
        self.assertFalse(self.observer.ready())
        self.respond('collision_monitor', 3)
        self.assertTrue(self.observer.ready())

    def test_missing_service_does_not_reuse_old_active_state(self):
        self.assertFalse(self.observer.ready())
        for name in NAV2_LIFECYCLE_NODES:
            self.respond(name, 3)
        self.assertTrue(self.observer.ready())
        self.node.clients['bt_navigator'].available = False
        self.now += 0.3
        self.assertFalse(self.observer.ready())
        for name in NAV2_LIFECYCLE_NODES:
            if name != 'bt_navigator':
                self.respond(name, 3)
        self.assertFalse(self.observer.ready())


if __name__ == '__main__':
    unittest.main()
