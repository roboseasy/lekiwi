from pathlib import Path
import importlib.util
import importlib.machinery
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_probe_module():
    script_path = PACKAGE_ROOT / "scripts" / "scan_tf_timing_probe"
    loader = importlib.machinery.SourceFileLoader("scan_tf_timing_probe", str(script_path))
    spec = importlib.util.spec_from_loader("scan_tf_timing_probe", loader)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_recommend_offset_uses_largest_tf_lag_with_margin() -> None:
    probe = load_probe_module()

    samples = [
        probe.TimingSample(scan_stamp_sec=10.00, node_now_sec=10.04, latest_tf_stamp_sec=9.98, tf_available=False),
        probe.TimingSample(scan_stamp_sec=11.00, node_now_sec=11.03, latest_tf_stamp_sec=10.95, tf_available=False),
        probe.TimingSample(scan_stamp_sec=12.00, node_now_sec=12.02, latest_tf_stamp_sec=12.01, tf_available=True),
    ]

    hint = probe.suggest_tf_time_offset(samples, margin_sec=0.01)

    assert hint.recommended_offset_sec == 0.06
    assert hint.max_tf_lag_sec == 0.05
    assert hint.failed_lookup_count == 2
    assert hint.successful_lookup_count == 1


def test_recommend_offset_is_zero_when_tf_is_not_behind_scan() -> None:
    probe = load_probe_module()

    samples = [
        probe.TimingSample(scan_stamp_sec=20.00, node_now_sec=20.02, latest_tf_stamp_sec=20.01, tf_available=True),
        probe.TimingSample(scan_stamp_sec=21.00, node_now_sec=21.02, latest_tf_stamp_sec=21.00, tf_available=True),
    ]

    hint = probe.suggest_tf_time_offset(samples)

    assert hint.recommended_offset_sec == 0.0
    assert hint.max_tf_lag_sec == 0.0
    assert hint.failed_lookup_count == 0
    assert hint.successful_lookup_count == 2


def test_script_declares_expected_parameter_defaults() -> None:
    script = (PACKAGE_ROOT / "scripts" / "scan_tf_timing_probe").read_text(encoding="utf-8")

    for expected in (
        "declare_parameter('scan_topic', '/scan')",
        "declare_parameter('target_frame', 'odom')",
        "declare_parameter('source_frame', '')",
        "declare_parameter('sample_count', 20)",
    ):
        assert expected in script


def test_script_handles_external_shutdown_without_traceback() -> None:
    script = (PACKAGE_ROOT / "scripts" / "scan_tf_timing_probe").read_text(encoding="utf-8")

    assert "ExternalShutdownException" in script
    assert "except (KeyboardInterrupt, ExternalShutdownException):" in script


def test_cmake_installs_probe_executable() -> None:
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "scripts/scan_tf_timing_probe" in cmake
    assert "DESTINATION lib/${PROJECT_NAME}" in cmake


if __name__ == "__main__":
    test_recommend_offset_uses_largest_tf_lag_with_margin()
    test_recommend_offset_is_zero_when_tf_is_not_behind_scan()
    test_script_declares_expected_parameter_defaults()
    test_script_handles_external_shutdown_without_traceback()
    test_cmake_installs_probe_executable()
