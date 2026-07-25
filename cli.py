import sys
import argparse

import numpy as np

from urdf_parser import URDFParser
from kinematics import forward_kinematics, compute_forward_kinematics_full
from jacobians import compute_jacobian, validate_jacobian_numerically
from inverse_kinematics import inverse_kinematics

# Set numpy printing to be more readable
np.set_printoptions(suppress=True, precision=4)


def build_arg_parser():
    parser_cli = argparse.ArgumentParser(
        description="RoboTool CLI - Forward/Inverse Kinematics from URDF"
    )
    parser_cli.add_argument(
        "urdf_file",
        help="Path to the URDF file describing the robot",
    )
    parser_cli.add_argument(
        "--target",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="IK target position in meters",
    )
    parser_cli.add_argument(
        "--target-rpy",
        nargs=3,
        type=float,
        metavar=("ROLL", "PITCH", "YAW"),
        default=None,
        help="Optional IK target orientation in radians (roll, pitch, yaw). "
             "Requires --target. Only used if inverse_kinematics() supports "
             "orientation targets.",
    )
    return parser_cli


def load_robot(urdf_file):
    """Parse the URDF file, exiting with a clear message on failure."""
    parser = URDFParser()
    try:
        return parser.parse(urdf_file)
    except FileNotFoundError:
        print(f"Error: URDF file not found: {urdf_file}")
        sys.exit(1)
    except Exception as exc:
        print(f"Error: failed to parse URDF file '{urdf_file}': {exc}")
        sys.exit(1)


def run_forward_kinematics(robot, q):
    fk_result = forward_kinematics(robot, q)
    jacobians = compute_jacobian(robot, q)
    J_analytic, J_numeric = validate_jacobian_numerically(robot, q)

    print("Forward Kinematics Result (Transformation Matrix):")
    print(fk_result)
    print("\nJacobians:")
    print(jacobians)
    print("Max Jv error:", np.max(np.abs(J_analytic[:3] - J_numeric[:3])))
    print("Max Jw error:", np.max(np.abs(J_analytic[3:] - J_numeric[3:])))


def run_inverse_kinematics(robot, q, target, target_rpy):
    ik_kwargs = dict(target_pos=target, initial_guess=q)

    if target_rpy is not None:
        try:
            ik_result = inverse_kinematics(
                robot, target_orientation=np.array(target_rpy), **ik_kwargs
            )
        except TypeError:
            print(
                "Warning: inverse_kinematics() does not currently accept "
                "'target_orientation'; solving position-only IK instead. "
                "Update inverse_kinematics() to add orientation support."
            )
            ik_result = inverse_kinematics(robot, **ik_kwargs)
    else:
        ik_result = inverse_kinematics(robot, **ik_kwargs)

    final_pos = compute_forward_kinematics_full(robot, ik_result)[-1][:3, 3]

    print("\nInverse Kinematics Result:")
    print(ik_result)
    print(f"Target:         {np.round(target, 4)}")
    print(f"Achieved:       {np.round(final_pos, 4)}")
    print(f"Position error: {np.linalg.norm(target - final_pos):.6f} m")


def main():
    parser_cli = build_arg_parser()
    args = parser_cli.parse_args()

    if args.target_rpy is not None and args.target is None:
        parser_cli.error("--target-rpy requires --target to also be specified")

    robot = load_robot(args.urdf_file)

    q = np.zeros(len([j for j in robot.joints if j.joint_type != "fixed"]))

    run_forward_kinematics(robot, q)

    if args.target:
        target = np.array(args.target)
        run_inverse_kinematics(robot, q, target, args.target_rpy)


if __name__ == "__main__":
    main()