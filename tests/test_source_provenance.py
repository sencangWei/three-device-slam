import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROVENANCE = ROOT / "provenance" / "source_files.json"
D405_MAIN = "a7a143df9a138ada481e6c234b03803ab0cae837"
MIGRATION = "de2b52f40b493d0f5ee63bbeca8d4afcb1a8cb78"


def test_frozen_sources_are_explicit():
    payload = json.loads(PROVENANCE.read_text(encoding="utf-8"))
    assert payload["schema"] == "three-device-slam.source-provenance.v1"
    assert payload["sources"]["d405_product_main"]["commit"] == D405_MAIN
    assert payload["sources"]["three_device_migration"]["commit"] == MIGRATION
    assert payload["files"]


def test_source_map_has_unique_targets():
    payload = json.loads(PROVENANCE.read_text(encoding="utf-8"))
    targets = [row["target"] for row in payload["files"]]
    assert len(targets) == len(set(targets))


def test_runtime_never_references_sibling_d405_repo():
    forbidden = ("D405-MAXIMU", ".worktrees/three-device-acquisition")
    for path in (ROOT / "three_device_slam").rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        assert not any(value in text for value in forbidden), path
