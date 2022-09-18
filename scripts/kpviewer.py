#!/usr/bin/env python3
from typing import Any

import kinpy as kp


def main(args: Any) -> None:
    chain = kp.build_chain_from_file(args.filename)
    print(chain)
    if args.show3d:
        param_names = chain.get_joint_parameter_names()
        th = {name: 0.0 for name in param_names}
        ret = chain.forward_kinematics(th)
        viz = kp.Visualizer()
        viz.add_robot(ret, chain.visuals_map())
        viz.spin()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Robot model viewer.")
    parser.add_argument("filename", type=str, help="Robot model filename.")
    parser.add_argument("--show3d", action="store_true", help="Show 3D model on GUI window.")
    args = parser.parse_args()
    main(args)
