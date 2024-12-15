import vamp
import numpy as np

def main(configs_path: str, results_path: str):
    cfgs = None
    with open(configs_path, "r") as f:
        cfgs = np.genfromtxt(f, delimiter=",")

    results = None
    with open(results_path, "r") as f:
        results = np.genfromtxt(f, dtype=bool, delimiter="\n")

    assert(results is not None)
    assert(cfgs is not None)

    sphere_centers = [
        [0.55, 0.0, 0.25],
        [0.35, 0.35, 0.25],
        [0.0, 0.55, 0.25],
        [-0.55, 0.0, 0.25],
        [-0.35, -0.35, 0.25],
        [0.0, -0.55, 0.25],
        [0.35, -0.35, 0.25],
        [0.35, 0.35, 0.8],
        [0.0, 0.55, 0.8],
        [-0.35, 0.35, 0.8],
        [-0.55, 0.0, 0.8],
        [-0.35, -0.35, 0.8],
        [0.0, -0.55, 0.8],
        [0.35, -0.35, 0.8],
    ]

    r = 0.2

    env = vamp.Environment()
    for pos in sphere_centers:
        env.add_sphere(vamp.Sphere(pos, r))

    for (q, expected) in zip(cfgs, results):
        res = vamp.panda.validate(q, env)
        if res != expected:
            print(f"q={q}, rumple={expected}, res={res}")

if __name__ == "__main__":
    from fire import Fire
    Fire(main)
