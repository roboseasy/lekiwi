import importlib
from types import SimpleNamespace
from unittest.mock import patch


module = importlib.import_module('lekiwi_teleop.guarded_base_teleop')


class FakeWindow:
    def __init__(self):
        self.timers = {}
        self.next_timer = 0
        self.cancelled = set()
        self.quit_called = False

    def bind(self, *_args):
        pass

    def protocol(self, *_args):
        pass

    def after(self, _delay, callback):
        self.next_timer += 1
        self.timers[self.next_timer] = callback
        return self.next_timer

    def after_cancel(self, timer):
        self.cancelled.add(timer)
        self.timers.pop(timer, None)

    def fire(self, timer):
        self.timers.pop(timer)()

    def quit(self):
        self.quit_called = True


class FakeEndpoint:
    def __init__(self):
        self.node = object()
        self.motor_ready = True
        self.published = []

    def publish_motion(self, motion):
        self.published.append(motion)


def key(name):
    return SimpleNamespace(keysym=name)


def test_release_stops_and_number_keys_choose_speed():
    profile = module.HeldKeySpeedProfile()
    profile.press('3')
    profile.press('w')
    for _ in range(20):
        motion = profile.next_motion()
    assert motion == module.Motion(linear_x=0.06)
    assert profile.release('w')
    assert profile.next_motion() == module.Motion()
    profile.press('1')
    profile.press('q')
    for _ in range(5):
        motion = profile.next_motion()
    assert motion == module.Motion(angular_z=0.23)
    profile.press('space')
    assert profile.next_motion() == module.Motion()
    assert profile.speed_stage == 1


def test_direction_switch_and_return_to_held_key_insert_zero():
    profile = module.HeldKeySpeedProfile()
    profile.press('w')
    assert profile.next_motion().linear_x > 0
    profile.press('a')
    assert profile.next_motion() == module.Motion()
    assert profile.next_motion().linear_y > 0
    assert profile.release('a')
    assert profile.next_motion() == module.Motion()
    assert profile.next_motion().linear_x > 0


def test_key_release_autorepeat_and_focus_loss_stop():
    root = FakeWindow()
    endpoint = FakeEndpoint()
    window = module.HoldTeleopWindow(root, endpoint, deadline=100.0,
                                     clock=lambda: 0.0)
    with patch.object(module.rclpy, 'spin_once'):
        window.on_press(key('w'))
        window.tick()
        assert endpoint.published[-1].linear_x > 0

        window.on_release(key('w'))
        pending = window.pending_releases['w']
        window.on_press(key('w'))  # X11 auto-repeat release/press pair.
        assert pending in root.cancelled
        assert window.profile.active_key == 'w'

        window.on_release(key('w'))
        root.fire(window.pending_releases['w'])
        assert endpoint.published[-1] == module.Motion()
        assert window.profile.active_key is None

        window.on_press(key('a'))
        window.tick()
        assert endpoint.published[-1].linear_y > 0
        window.on_focus_out(None)
        assert endpoint.published[-1] == module.Motion()
        assert window.profile.active_key is None
        window.on_press(key('a'))
        assert window.profile.active_key is None  # A held across focus loss cannot restart.
        window.on_release(key('a'))
        root.fire(window.pending_releases['a'])
        window.on_press(key('a'))
        assert window.profile.active_key == 'a'
        window.on_press(key('space'))
        assert endpoint.published[-1] == module.Motion()
        window.on_press(key('a'))
        assert window.profile.active_key is None  # Space remains stopped until release.
        window.on_release(key('a'))
        root.fire(window.pending_releases['a'])

        window.on_press(key('e'))
        window.on_close()
        assert endpoint.published[-1] == module.Motion()
    assert root.quit_called


def test_close_leaves_event_loop_even_if_zero_publish_fails():
    root = FakeWindow()
    endpoint = FakeEndpoint()
    window = module.HoldTeleopWindow(root, endpoint, deadline=100.0,
                                     clock=lambda: 0.0)

    def fail_publish(_motion):
        raise RuntimeError('publisher unavailable')

    endpoint.publish_motion = fail_publish
    try:
        window.on_close()
    except RuntimeError as error:
        assert str(error) == 'publisher unavailable'
    else:
        raise AssertionError('expected publish failure')
    assert window.closed
    assert root.quit_called
