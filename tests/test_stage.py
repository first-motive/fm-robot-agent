"""Staging the Anvil bundle from a synthetic fm-skills checkout. No robot."""

from __future__ import annotations

from pathlib import Path

import pytest

from fm_robot_agent import stage
from fm_robot_agent.description import BUNDLE_ENV, DescriptionBundle

PACKAGE_URI = "package://openarm_v2_anvil_description/cad/arms/link1.stl"
URDF = f'<robot name="anvil"><link name="follower_l_link1"><mesh filename="{PACKAGE_URI}"/></link></robot>'


@pytest.fixture
def checkout(tmp_path):
    package = tmp_path / "fm-skills" / "src" / stage.ANVIL_PACKAGE
    (package / "urdf").mkdir(parents=True)
    (package / "urdf" / "robot.urdf").write_text(URDF)
    (package / "cad" / "arms").mkdir(parents=True)
    (package / "cad" / "arms" / "link1.stl").write_bytes(b"solid link1")
    (package / "tools").mkdir()
    (package / "tools" / "generate.py").write_text("")
    return tmp_path / "fm-skills"


@pytest.fixture
def bundle(tmp_path):
    root = tmp_path / "bundle"
    (root / "arms").mkdir(parents=True)
    (root / "arms" / "old.stl").write_bytes(b"solid old")
    (root / "robot.urdf").write_text("<robot name='old'/>")
    (root / "calibration").mkdir()
    (root / "calibration" / "cam_chest.json").write_text("{}")
    return root


def test_the_package_urdf_and_meshes_are_served(checkout, bundle):
    stage.stage_anvil(checkout, bundle)
    served = DescriptionBundle(bundle)
    assert served.urdf() == URDF.encode()
    assert served.asset(PACKAGE_URI) == b"solid link1"


def test_stale_content_goes_and_other_entries_stay(checkout, bundle):
    (bundle / "grippers").mkdir()
    (bundle / "notes.txt").write_text("operator's")
    stage.stage_anvil(checkout, bundle)
    assert sorted(entry.name for entry in bundle.iterdir()) == [
        "calibration",
        "notes.txt",
        "packages",
        "robot.urdf",
    ]
    assert (bundle / "calibration" / "cam_chest.json").read_text() == "{}"
    assert (bundle / "notes.txt").read_text() == "operator's"
    package = bundle / "packages" / stage.ANVIL_PACKAGE
    assert sorted(entry.name for entry in package.iterdir()) == ["cad", "urdf"]


def test_no_staging_directory_is_left_beside_the_bundle(checkout, bundle):
    stage.stage_anvil(checkout, bundle)
    assert sorted(entry.name for entry in bundle.parent.iterdir()) == ["bundle", "fm-skills"]


def test_a_checkout_without_the_package_changes_nothing(tmp_path, bundle):
    with pytest.raises(stage.StageError, match="missing urdf/robot.urdf"):
        stage.stage_anvil(tmp_path / "elsewhere", bundle)
    assert (bundle / "arms" / "old.stl").exists()


def test_a_failed_copy_leaves_the_old_bundle(checkout, bundle, monkeypatch):
    def fail(*args, **kwargs):
        raise OSError("disk full")

    monkeypatch.setattr(stage.shutil, "copytree", fail)
    with pytest.raises(OSError):
        stage.stage_anvil(checkout, bundle)
    assert (bundle / "robot.urdf").read_text() == "<robot name='old'/>"
    assert (bundle / "arms" / "old.stl").read_bytes() == b"solid old"
    assert sorted(entry.name for entry in bundle.parent.iterdir()) == ["bundle", "fm-skills"]


def test_a_failed_swap_puts_the_old_entries_back(checkout, bundle, monkeypatch):
    rename = Path.rename

    def fail_on_packages(self, target):
        if Path(target).name == "packages":
            raise OSError("cross-device")
        return rename(self, target)

    monkeypatch.setattr(Path, "rename", fail_on_packages)
    with pytest.raises(OSError):
        stage.stage_anvil(checkout, bundle)
    assert sorted(entry.name for entry in bundle.iterdir()) == ["arms", "calibration", "robot.urdf"]
    assert (bundle / "robot.urdf").read_text() == "<robot name='old'/>"


def test_a_symlink_in_the_package_is_refused(checkout, bundle, tmp_path):
    secret = tmp_path / "secret.stl"
    secret.write_bytes(b"not the package's")
    (checkout / "src" / stage.ANVIL_PACKAGE / "cad" / "arms" / "leak.stl").symlink_to(secret)
    with pytest.raises(stage.StageError, match="is a symlink"):
        stage.stage_anvil(checkout, bundle)
    assert (bundle / "arms" / "old.stl").exists()


@pytest.mark.parametrize("where", ["home", "root", "checkout", "above_checkout"])
def test_a_bundle_path_that_is_not_a_bundle_is_refused(checkout, monkeypatch, tmp_path, where):
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    (tmp_path / "home").mkdir()
    target = {
        "home": tmp_path / "home",
        "root": Path("/"),
        "checkout": checkout,
        "above_checkout": tmp_path,
    }[where]
    with pytest.raises(stage.StageError):
        stage.stage_anvil(checkout, target)
    assert (checkout / "src" / stage.ANVIL_PACKAGE / "urdf" / "robot.urdf").exists()


def test_main_stages_into_the_configured_bundle(checkout, bundle, monkeypatch, capsys):
    monkeypatch.setenv(BUNDLE_ENV, str(bundle))
    assert stage.main([str(checkout)]) == 0
    assert (bundle / "robot.urdf").read_text() == URDF
    assert stage.ANVIL_PACKAGE in capsys.readouterr().out


def test_main_exits_3_when_the_checkout_has_no_package(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv(BUNDLE_ENV, str(tmp_path / "bundle"))
    assert stage.main([str(tmp_path)]) == 3
    assert "missing" in capsys.readouterr().err
