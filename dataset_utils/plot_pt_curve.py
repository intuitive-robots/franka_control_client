import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import torch


def load_pt(path, key=None):
    pt = torch.load(path, map_location="cpu")

    if key is not None:
        if not isinstance(pt, dict):
            raise TypeError("--key can only be used when the .pt file contains a dict")
        if key not in pt:
            raise KeyError(f"Key {key!r} not found. Available keys: {list(pt.keys())}")
        pt = pt[key]
    elif isinstance(pt, dict):
        tensor_items = [(name, value) for name, value in pt.items() if torch.is_tensor(value)]
        if len(tensor_items) != 1:
            keys = [name for name, _ in tensor_items]
            raise ValueError(
                "The .pt file contains multiple/no top-level tensors. "
                f"Use --key. Tensor keys: {keys}"
            )
        pt = tensor_items[0][1]

    return torch.as_tensor(pt).detach().cpu().float()


def make_curve_array(tensor):
    if tensor.ndim == 0:
        return tensor.reshape(1, 1)
    if tensor.ndim == 1:
        return tensor.reshape(-1, 1)
    return tensor.reshape(tensor.shape[0], -1)


def plot_curves(curves, output_path, title=None, show=False):
    time_steps, dims = curves.shape
    fig_height = max(3.0, min(18.0, 1.8 * dims))
    fig, axes = plt.subplots(dims, 1, sharex=True, figsize=(10, fig_height), squeeze=False)

    x = range(time_steps)
    for dim in range(dims):
        ax = axes[dim][0]
        ax.plot(x, curves[:, dim].numpy(), linewidth=1.5)
        ax.set_ylabel(f"dim {dim}")
        ax.grid(True, alpha=0.3)

    axes[-1][0].set_xlabel("index")
    fig.suptitle(title or "PT curves")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)

    if show:
        plt.show()

    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(description="Plot each dimension in a .pt tensor as a curve.")
    parser.add_argument("pt_path", help="Path to the .pt file")
    parser.add_argument("--key", help="Top-level key to read if the .pt file contains a dict")
    parser.add_argument(
        "-o",
        "--output",
        help="Output image path. Defaults to '<pt filename>_curve.png' next to the input file.",
    )
    parser.add_argument("--show", action="store_true", help="Open an interactive plot window")
    return parser.parse_args()


def main():
    args = parse_args()
    pt_path = Path(args.pt_path).expanduser()
    output_path = (
        Path(args.output).expanduser()
        if args.output
        else pt_path.with_name(f"{pt_path.stem}_curve.png")
    )

    tensor = load_pt(pt_path, key=args.key)
    curves = make_curve_array(tensor)
    plot_curves(curves, output_path, title=f"{pt_path.name} shape={tuple(tensor.shape)}", show=args.show)

    print(f"Loaded shape: {tuple(tensor.shape)}")
    print(f"Plotted dimensions: {curves.shape[1]}")
    print(f"Saved: {output_path}")


if __name__ == "__main__":
    main()
