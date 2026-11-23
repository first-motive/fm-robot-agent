"""The lift height file: what is accepted, what is read back, and how it is written."""

import os

import pytest

from fm_robot_agent.lift import (
    LIFT_TRAVEL_MM,
    MAX_FILE_BYTES,
    parse_height,
    read_height,
    write_height,
)


@pytest.mark.parametrize(("value", "height"), [("0", 0.0), ("310.5", 310.5), ("457.2", LIFT_TRAVEL_MM)])
def test_a_reading_inside_the_travel_is_parsed(value, height):
    assert parse_height(value) == height


@pytest.mark.parametrize("value", ["-0.1", "457.3", "nan", "inf", "low", "", None, [310]])
def test_a_reading_outside_the_travel_or_not_a_number_is_refused(value):
    with pytest.raises(ValueError):
        parse_height(value)


def test_no_file_reads_as_fully_raised(tmp_path):
    assert read_height(tmp_path / "height_mm") == LIFT_TRAVEL_MM


def test_an_empty_file_reads_as_fully_raised(tmp_path):
    (tmp_path / "height_mm").write_text("\n")
    assert read_height(tmp_path / "height_mm") == LIFT_TRAVEL_MM


@pytest.mark.parametrize("text", ["lowered", "900", "1" * (MAX_FILE_BYTES + 1)])
def test_a_file_that_holds_no_reading_reads_as_none(tmp_path, text):
    (tmp_path / "height_mm").write_text(text)
    assert read_height(tmp_path / "height_mm") is None


def test_a_directory_at_the_path_reads_as_none(tmp_path):
    (tmp_path / "height_mm").mkdir()
    assert read_height(tmp_path / "height_mm") is None


def test_a_link_at_the_path_reads_as_none(tmp_path):
    (tmp_path / "elsewhere").write_text("310\n")
    (tmp_path / "height_mm").symlink_to(tmp_path / "elsewhere")
    assert read_height(tmp_path / "height_mm") is None


def test_an_unreadable_file_reads_as_none(tmp_path):
    path = tmp_path / "height_mm"
    path.write_bytes(b"\xff\xfe")
    assert read_height(path) is None


def test_a_refusal_quotes_only_the_start_of_a_long_value():
    with pytest.raises(ValueError) as refused:
        parse_height("x" * 10_000)
    assert len(str(refused.value)) < 100


def test_a_failed_rename_leaves_no_temporary_file(tmp_path, monkeypatch):
    def refuse(*_):
        raise OSError("refused")

    monkeypatch.setattr(os, "replace", refuse)
    with pytest.raises(OSError):
        write_height(tmp_path / "height_mm", 310.0)
    assert os.listdir(tmp_path) == []


def test_a_written_reading_reads_back(tmp_path):
    path = tmp_path / "height_mm"
    write_height(path, 310.5)
    assert path.read_text() == "310.5\n"
    assert read_height(path) == 310.5
    assert oct(path.stat().st_mode & 0o777) == "0o644"


def test_a_write_leaves_no_temporary_file(tmp_path):
    write_height(tmp_path / "height_mm", 310.0)
    assert os.listdir(tmp_path) == ["height_mm"]


def test_a_write_replaces_a_link_at_the_path_rather_than_following_it(tmp_path):
    target = tmp_path / "elsewhere"
    target.write_text("keep\n")
    (tmp_path / "height_mm").symlink_to(target)
    write_height(tmp_path / "height_mm", 310.0)
    assert target.read_text() == "keep\n"
    assert not (tmp_path / "height_mm").is_symlink()


def test_a_missing_directory_raises(tmp_path):
    with pytest.raises(OSError):
        write_height(tmp_path / "absent" / "height_mm", 310.0)
