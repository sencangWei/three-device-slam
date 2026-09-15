"""Shared pytest fixtures for repository-local test isolation."""

import pytest


@pytest.fixture(autouse=True)
def isolate_common_board_artifacts(request):
    """Keep common-board output tests inside their per-test temporary tree."""
    if not request.node.path.name.startswith("test_common_board_"):
        return

    tmp_path = request.getfixturevalue("tmp_path")
    monkeypatch = request.getfixturevalue("monkeypatch")
    from scripts import common_board_calibration as calibration

    monkeypatch.setattr(calibration, "ARTIFACTS_ROOT", tmp_path.resolve())
