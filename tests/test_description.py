"""The description bundle and its two read verbs. No router, no robot."""

from __future__ import annotations

import json

import pytest

from fm_robot_agent import description
from fm_robot_agent.description import DescriptionBundle
from fm_robot_agent.fake import FakeAdapter
from fm_robot_agent.verbs import KEY_PREFIX, answer, verb_of

NS = "fm_rob_01"
URDF = b'<robot name="arm"><link name="base"/></robot>'


@pytest.fixture
def bundle(tmp_path) -> DescriptionBundle:
    (tmp_path / "robot.urdf").write_bytes(URDF)
    meshes = tmp_path / "packages" / "assembly" / "meshes"
    meshes.mkdir(parents=True)
    (meshes / "Base.stl").write_bytes(b"solid base")
    (tmp_path / "assets").mkdir()
    (tmp_path / "assets" / "jaw.stl").write_bytes(b"solid jaw")
    (tmp_path / "packages" / "notes.txt").write_text("not a mesh")
    return DescriptionBundle(tmp_path)


def ask(bundle, verb, parameters=""):
    return answer(f"{KEY_PREFIX}/{NS}/{verb}", FakeAdapter(), NS, parameters=parameters, bundle=bundle)


def test_description_serves_the_urdf_as_text(bundle):
    reply = ask(bundle, "description")
    assert reply.ok and reply.payload == URDF and reply.encoding == "text/xml"


def test_a_package_mesh_is_served_as_bytes(bundle):
    reply = ask(bundle, "asset", "uri=package://assembly/meshes/Base.stl")
    assert reply.ok and reply.payload == b"solid base"
    assert reply.encoding == "application/octet-stream"


def test_a_percent_encoded_uri_is_the_same_mesh(bundle):
    reply = ask(bundle, "asset", "uri=package%3A%2F%2Fassembly%2Fmeshes%2FBase.stl")
    assert reply.ok and reply.payload == b"solid base"


def test_a_relative_mesh_resolves_inside_the_bundle(bundle):
    assert ask(bundle, "asset", "uri=assets/jaw.stl").payload == b"solid jaw"


@pytest.mark.parametrize(
    "uri",
    [
        "package://assembly/../../../etc/passwd.stl",
        "../outside.stl",
        "/etc/hosts.stl",
        "file:///etc/hosts.stl",
        "package://notes.txt",
        "package://assembly/meshes/Missing.stl",
        "",
    ],
)
def test_an_asset_outside_the_bundle_or_not_a_mesh_is_refused(bundle, uri):
    reply = ask(bundle, "asset", f"uri={uri}")
    assert reply.ok is False
    assert json.loads(reply.payload)["ok"] is False


def test_a_symlink_cannot_reach_past_the_bundle(bundle, tmp_path_factory):
    outside = tmp_path_factory.mktemp("outside") / "secret.stl"
    outside.write_bytes(b"secret")
    (bundle.root / "packages" / "assembly" / "meshes" / "link.stl").symlink_to(outside)
    assert ask(bundle, "asset", "uri=package://assembly/meshes/link.stl").ok is False


def test_an_oversized_asset_is_refused(bundle, monkeypatch):
    monkeypatch.setattr(description, "MAX_ASSET_BYTES", 4)
    assert ask(bundle, "asset", "uri=package://assembly/meshes/Base.stl").ok is False


def test_no_staged_description_names_the_file_not_the_host_path(tmp_path):
    reply = ask(DescriptionBundle(tmp_path / "missing"), "description")
    assert reply.ok is False
    error = json.loads(reply.payload)["error"]
    assert "robot.urdf" in error and str(tmp_path) not in error


def test_an_oversized_urdf_is_refused(bundle, monkeypatch):
    monkeypatch.setattr(description, "MAX_URDF_BYTES", 4)
    assert ask(bundle, "description").ok is False


def test_a_urdf_linked_out_of_the_bundle_is_not_served(tmp_path, tmp_path_factory):
    outside = tmp_path_factory.mktemp("outside") / "passwd"
    outside.write_text("root:x:0:0")
    (tmp_path / "robot.urdf").symlink_to(outside)
    assert ask(DescriptionBundle(tmp_path), "description").ok is False


def test_a_mesh_named_link_to_a_non_mesh_is_refused(bundle):
    """The suffix is checked on the file reached, not on the name asked for."""
    (bundle.root / "packages" / "assembly" / "meshes" / "urdf.stl").symlink_to(bundle.root / "robot.urdf")
    assert ask(bundle, "asset", "uri=package://assembly/meshes/urdf.stl").ok is False


def test_a_third_concurrent_asset_read_is_refused(bundle, monkeypatch):
    import threading

    monkeypatch.setattr(description, "_asset_slots", threading.BoundedSemaphore(0))
    reply = ask(bundle, "asset", "uri=package://assembly/meshes/Base.stl")
    assert reply.ok is False
    assert "busy" in json.loads(reply.payload)["error"]


def test_an_unreadable_mesh_is_a_refusal_not_a_dropped_query(bundle):
    mesh = bundle.root / "packages" / "assembly" / "meshes" / "Base.stl"
    mesh.chmod(0)
    try:
        assert ask(bundle, "asset", "uri=package://assembly/meshes/Base.stl").ok is False
    finally:
        mesh.chmod(0o644)


@pytest.mark.parametrize("verb", ["description", "asset"])
def test_a_wildcard_cannot_pull_descriptions(verb):
    """Discovery reads status; it has no use for every robot's meshes."""
    assert verb_of(f"{KEY_PREFIX}/*/{verb}", NS) is None
    assert verb_of(f"{KEY_PREFIX}/{NS}/{verb}", NS) == verb


def test_the_bundle_dir_follows_its_override(monkeypatch, tmp_path):
    monkeypatch.setenv(description.BUNDLE_ENV, str(tmp_path))
    assert description.bundle_dir() == tmp_path
