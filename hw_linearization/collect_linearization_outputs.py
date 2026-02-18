"""Collect outputs from D/E/F linearization scripts into a single text file."""

from __future__ import annotations

import io
import os
import runpy
import sys
from contextlib import redirect_stdout
from pathlib import Path


def main() -> None:
    here = Path(__file__).resolve().parent

    scripts = [
        here / "D_mass_D4_D6_linearization.py",
        here / "E_blockbeam_E4_E6_linearization.py",
        here / "F_vtol_F4_F6_linearization.py",
    ]

    output_file = here / "linearization_outputs.txt"

    buffer = io.StringIO()

    for script in scripts:
        if not script.exists():
            raise FileNotFoundError(f"Missing script: {script}")

        buffer.write("\n" + "=" * 80 + "\n")
        buffer.write(f"RUNNING: {script.name}\n")
        buffer.write("=" * 80 + "\n\n")

        with redirect_stdout(buffer):
            runpy.run_path(str(script), run_name="__main__")

        buffer.write("\n")

    output_file.write_text(buffer.getvalue(), encoding="utf-8")
    print(f"Wrote output to: {output_file}")


if __name__ == "__main__":
    # Ensure local imports inside scripts resolve correctly
    sys.path.insert(0, os.fspath(Path(__file__).resolve().parent))
    main()
