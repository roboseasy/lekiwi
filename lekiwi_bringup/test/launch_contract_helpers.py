"""Inspect generated launch actions without executing nodes or hardware."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import TextSubstitution
from launch.utilities import normalize_to_list_of_substitutions


def load_launch(name):
    path = Path(__file__).resolve().parents[1] / 'launch' / name
    spec = importlib.util.spec_from_file_location('lekiwi_' + name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def declared_launch_arguments(name):
    return {action.name: action for action in load_launch(name).generate_launch_description().entities
            if isinstance(action, DeclareLaunchArgument)}


def launch_argument_defaults(name):
    return {name: ''.join(value.text for value in action.default_value)
            for name, action in declared_launch_arguments(name).items()
            if all(isinstance(value, TextSubstitution) for value in action.default_value)}


def forwarded_base_arguments():
    includes = [action for action in load_launch('robot.launch.py').generate_launch_description().entities
                if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == 1
    return {name: normalize_to_list_of_substitutions(value)
            for name, value in includes[0].launch_arguments}
