"""Check a standalone runtime install and explicit developer diagnostics install."""

from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
DIAGNOSTICS = ('test_all.py', 'test_ros_topics.py', 'test_service_server.py')


class TestInstallLayout(unittest.TestCase):
    def install(self, folder, diagnostics=False):
        source = folder / 'source'
        source.mkdir()
        for name in ('CMakeLists.txt', 'package.xml'):
            shutil.copy2(PACKAGE / name, source / name)
        for name in ('launch', 'param', 'lekiwi_bringup'):
            shutil.copytree(PACKAGE / name, source / name,
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        if diagnostics:
            shutil.copytree(PACKAGE / 'tools', source / 'tools',
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        # A runtime package must configure without repository-level test/tool files.
        (source / 'launch/__pycache__').mkdir()
        (source / 'launch/__pycache__/stale.pyc').write_bytes(b'not for distribution')
        (source / 'launch/stale.pyc').write_bytes(b'not for distribution')
        build, prefix = folder / 'build', folder / 'install'
        configure = ['cmake', '-S', str(source), '-B', str(build), '-DBUILD_TESTING=OFF',
                     f'-DPython3_EXECUTABLE={sys.executable}', f'-DCMAKE_INSTALL_PREFIX={prefix}']
        if diagnostics:
            configure.append('-DLEKIWI_INSTALL_DIAGNOSTICS=ON')
        for command in (configure, ['cmake', '--build', str(build)],
                        ['cmake', '--install', str(build)]):
            result = subprocess.run(command, capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return prefix

    def check_runtime(self, prefix):
        share = prefix / 'share/lekiwi_bringup'
        for relative in ('launch/base.launch.py', 'launch/robot.launch.py',
                         'param/base.yaml', 'param/base_profile.example.yaml', 'package.xml'):
            self.assertTrue((share / relative).is_file(), relative)
        self.assertFalse(list(prefix.rglob('*.pyc')))
        self.assertFalse(list(prefix.rglob('__pycache__')))
        self.assertFalse((share / 'test').exists())
        modules = list(prefix.rglob('lekiwi_bringup/base_arguments.py'))
        self.assertEqual(len(modules), 1)
        env = {**os.environ, 'PYTHONDONTWRITEBYTECODE': '1',
               'PYTHONPATH': str(modules[0].parents[1]) + os.pathsep + os.environ.get('PYTHONPATH', '')}
        # Import the installed helper and construct both installed launches.
        # Construction is read-only; no launch service or ROS node is started.
        code = (
            'import runpy\n'
            'from launch.actions import DeclareLaunchArgument\n'
            'from lekiwi_bringup import base_arguments\n'
            f'assert base_arguments.__file__ == {str(modules[0])!r}\n'
            f'for name, count in (("base.launch.py", 30), ("robot.launch.py", 31)):\n'
            f'    module = runpy.run_path(str({str(share)!r} + "/launch/" + name))\n'
            '    actions = module["generate_launch_description"]().entities\n'
            '    assert sum(isinstance(a, DeclareLaunchArgument) for a in actions) == count\n'
        )
        result = subprocess.run([sys.executable, '-c', code], env=env, cwd=prefix,
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_default_install_has_runtime_without_developer_tools(self):
        with tempfile.TemporaryDirectory(prefix='lekiwi-runtime-install-') as temporary:
            prefix = self.install(Path(temporary))
            self.check_runtime(prefix)
            for name in DIAGNOSTICS:
                self.assertFalse((prefix / 'lib/lekiwi_bringup' / name).exists())
            self.assertFalse((prefix / 'share/lekiwi_bringup/launch/topic_test.launch.py').exists())

    def test_diagnostics_can_be_installed_explicitly(self):
        with tempfile.TemporaryDirectory(prefix='lekiwi-diagnostics-install-') as temporary:
            prefix = self.install(Path(temporary), diagnostics=True)
            self.check_runtime(prefix)
            for name in DIAGNOSTICS:
                script = prefix / 'lib/lekiwi_bringup' / name
                self.assertTrue(script.is_file(), name)
                self.assertTrue(script.stat().st_mode & 0o111, name)
            self.assertTrue((prefix / 'share/lekiwi_bringup/launch/topic_test.launch.py').is_file())


if __name__ == '__main__':
    unittest.main()
