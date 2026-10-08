import argparse
import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class CalibrationSample:
    current_scale: float
    odom_delta: float
    measured_delta: float

    @classmethod
    def from_args(cls, args: argparse.Namespace) -> "CalibrationSample":
        return cls(
            current_scale=args.current_scale,
            odom_delta=args.odom_delta,
            measured_delta=args.measured_delta,
        )


@dataclass(frozen=True)
class CalibrationRecommendation:
    recommended_scale: float
    odom_to_measured_ratio: float


def finite_positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError(f"0보다 큰 유한한 값이 필요합니다: {value}")
    return parsed


def finite_non_zero_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed == 0.0:
        raise argparse.ArgumentTypeError(f"0이 아닌 유한한 값이 필요합니다: {value}")
    return parsed


def calculate_scale_recommendation(sample: CalibrationSample) -> CalibrationRecommendation:
    if not math.isfinite(sample.current_scale) or sample.current_scale <= 0.0:
        raise ValueError("current_scale은 0보다 큰 유한한 값이어야 합니다.")
    if not math.isfinite(sample.odom_delta) or sample.odom_delta == 0.0:
        raise ValueError("odom_delta는 0이 아닌 유한한 값이어야 합니다.")
    if not math.isfinite(sample.measured_delta) or sample.measured_delta <= 0.0:
        raise ValueError("measured_delta는 0보다 큰 유한한 값이어야 합니다.")

    ratio = abs(sample.odom_delta) / sample.measured_delta
    return CalibrationRecommendation(
        recommended_scale=(
            sample.current_scale * sample.measured_delta / abs(sample.odom_delta)
        ),
        odom_to_measured_ratio=ratio,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "바닥 주행 실측값으로 encoder_odom_scale 후보를 계산합니다."
        )
    )
    parser.add_argument(
        "--current-scale",
        type=finite_positive_float,
        required=True,
        help="현재 encoder_odom_scale 값",
    )
    parser.add_argument(
        "--odom-delta",
        type=finite_non_zero_float,
        required=True,
        help="/odom 기준 이동거리 또는 회전각 변화량. 후진/시계 방향은 음수도 허용합니다.",
    )
    parser.add_argument(
        "--measured-delta",
        type=finite_positive_float,
        required=True,
        help="줄자, 각도계, 외부 기준으로 잰 실제 이동거리 또는 회전각의 절댓값",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    recommendation = calculate_scale_recommendation(CalibrationSample.from_args(args))

    print(
        "odom/measured ratio: "
        f"{recommendation.odom_to_measured_ratio:.6g}"
    )
    print(
        "recommended encoder_odom_scale: "
        f"{recommendation.recommended_scale:.6g}"
    )
    print(
        "launch argument: "
        "encoder_odom_scale:="
        f"{recommendation.recommended_scale:.6g}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
