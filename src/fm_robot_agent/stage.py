"""Stage the Anvil's description bundle from an fm-skills checkout.

Anvil's CAD model of the OpenArm v2 lives in the private fm-skills repo as the
ament package ``openarm_v2_anvil_description``. Its ``urdf/robot.urdf`` carries
the loader's joint and link names and names every mesh as
``package://openarm_v2_anvil_description/cad/...``, so the bundle the agent
serves (see ``description``) takes the package whole:

    <bundle>/robot.urdf                                       urdf/robot.urdf
    <bundle>/packages/openarm_v2_anvil_description/urdf/      urdf/
    <bundle>/packages/openarm_v2_anvil_description/cad/       cad/

Staging replaces only the entries it owns, plus ``arms/`` and ``grippers/``
that the earlier hand copy of the CAD export left. Anything else in the bundle,
``calibration/`` above all, is left as it is.

The checkout is a path the operator gives. Staging never clones or fetches, so
it needs no credentials for the private repo.

    fm robot stage ~/fm/fm-skills
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import tempfile
from pathlib import Path

from fm_robot_agent.description import PACKAGES_DIR, URDF_NAME, bundle_dir

ANVIL_PACKAGE = "openarm_v2_anvil_description"

#: The package's directories the URDF's package:// URIs can name.
PACKAGE_DIRS = ("urdf", "cad")

#: The bundle entries a restage replaces: its own, and the hand copy's.
MANAGED = (URDF_NAME, PACKAGES_DIR, "arms", "grippers")


class StageError(Exception):
    """Staging was refused before the bundle was changed."""


def anvil_package(checkout: Path) -> Path:
    """The package inside an fm-skills checkout, or raise when it is not there
    or holds a symlink."""
    package = checkout.expanduser() / "src" / ANVIL_PACKAGE
    missing = [name for name in ("urdf/robot.urdf", *PACKAGE_DIRS) if not (package / name).exists()]
    if missing:
        raise StageError(f"{package} is not an {ANVIL_PACKAGE} package: missing {', '.join(missing)}")
    # A link could carry a file from anywhere on this host into a bundle the
    # fabric reads, so the copy takes regular files only.
    for name in PACKAGE_DIRS:
        top = package / name
        if top.is_symlink():
            raise StageError(f"{top} is a symlink")
        for parent, dirs, files in os.walk(top):
            for entry in (*dirs, *files):
                if os.path.islink(os.path.join(parent, entry)):
                    raise StageError(f"{os.path.join(parent, entry)} is a symlink")
    return package


def check_bundle(bundle: Path, checkout: Path) -> None:
    """Refuse a bundle path whose managed entries are not the bundle's to replace."""
    target = bundle.expanduser().resolve()
    if target == Path.home().resolve() or target == Path(target.anchor):
        raise StageError(f"{target} is not a bundle directory")
    source = checkout.expanduser().resolve()
    if source == target or source.is_relative_to(target):
        raise StageError(f"{target} holds the checkout {source}")


def stage_anvil(checkout: Path, bundle: Path) -> None:
    """Replace the bundle's managed entries with the package's.

    The new entries are copied beside the bundle first, then swapped in. Should
    the swap fail, the old entries are put back, so the bundle is never left
    half staged.
    """
    package = anvil_package(checkout)
    check_bundle(bundle, checkout)
    bundle.mkdir(parents=True, exist_ok=True)
    incoming = Path(tempfile.mkdtemp(prefix=".stage-", dir=bundle.parent))
    try:
        new, old = incoming / "new", incoming / "old"
        old.mkdir()
        shutil.copytree(package / "urdf", new / PACKAGES_DIR / ANVIL_PACKAGE / "urdf")
        shutil.copytree(package / "cad", new / PACKAGES_DIR / ANVIL_PACKAGE / "cad")
        shutil.copy2(package / "urdf" / "robot.urdf", new / URDF_NAME)
        _swap(bundle, new, old)
    finally:
        shutil.rmtree(incoming, ignore_errors=True)


def _swap(bundle: Path, new: Path, old: Path) -> None:
    """Move the managed entries into ``old`` and ``new``'s into the bundle,
    undoing every move made so far if one fails."""
    moved: list[str] = []
    placed: list[str] = []
    try:
        for name in MANAGED:
            entry = bundle / name
            if entry.exists() or entry.is_symlink():
                entry.rename(old / name)
                moved.append(name)
        for entry in new.iterdir():
            entry.rename(bundle / entry.name)
            placed.append(entry.name)
    except OSError:
        for name in placed:
            if (bundle / name).is_dir():
                shutil.rmtree(bundle / name)
            else:
                (bundle / name).unlink()
        for name in moved:
            (old / name).rename(bundle / name)
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="fm robot stage",
        description=f"Stage the Anvil's description bundle from an fm-skills checkout's {ANVIL_PACKAGE}.",
    )
    parser.add_argument("checkout", type=Path, help="an fm-skills checkout on this host")
    args = parser.parse_args(argv)
    bundle = bundle_dir()
    try:
        stage_anvil(args.checkout, bundle)
    except StageError as error:
        print(f"fm robot stage: {error}", file=sys.stderr)
        return 3
    print(f"staged {ANVIL_PACKAGE} into {bundle}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
