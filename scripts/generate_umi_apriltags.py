#!/usr/bin/env python3
"""Generate exact-size official AprilTag 36h11 print assets for both UMIs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas


FAMILY = "tag36h11"
OFFICIAL_SOURCE_COMMIT = "f3fd9a7add5bfd82a886fc65240fdb8e3c9ac5a1"
EFFECTIVE_TAG_SIZE_MM = 40.0
CELL_SIZE_MM = EFFECTIVE_TAG_SIZE_MM / 8.0
FULL_GRID_SIZE_MM = CELL_SIZE_MM * 10.0
MM_TO_PT = 72.0 / 25.4
PNG_PIXELS_PER_CELL = 120
PNG_DPI = PNG_PIXELS_PER_CELL / CELL_SIZE_MM * 25.4

# Exact 10x10 pixels from AprilRobotics/apriltag-imgs.  The outer ring is the
# required white quiet zone; the inner 8x8 square is the measured tag size.
# "1" is black and "0" is white.
TAGS = {
    "left": {
        "id": 0,
        "official_png_sha256": "48d811f770fc3595aaf5650a5fd8007843e7fece1f8fdcb94e0f113b4c9ed0c6",
        "grid": (
            "0000000000",
            "0111111110",
            "0100101010",
            "0110001010",
            "0110011110",
            "0101011110",
            "0110100110",
            "0111101110",
            "0111111110",
            "0000000000",
        ),
    },
    "right": {
        "id": 1,
        "official_png_sha256": "2b30c316b3f8f32d80ba4e83feee8471c41a04aeba75bf1a23f1cf9c990394ce",
        "grid": (
            "0000000000",
            "0111111110",
            "0100100110",
            "0110100010",
            "0100001110",
            "0110011110",
            "0101001010",
            "0111011010",
            "0111111110",
            "0000000000",
        ),
    },
}


def _validate_grid(grid: tuple[str, ...]) -> None:
    if len(grid) != 10 or any(len(row) != 10 for row in grid):
        raise ValueError("official tag grid must be 10x10")
    if any(set(row) - {"0", "1"} for row in grid):
        raise ValueError("official tag grid must be binary")
    if any(any(value != "0" for value in grid[index]) for index in (0, 9)):
        raise ValueError("official tag grid must retain its white quiet zone")
    if any(row[0] != "0" or row[9] != "0" for row in grid):
        raise ValueError("official tag grid must retain its white quiet zone")
    if grid[1] != "0111111110" or grid[8] != "0111111110":
        raise ValueError("official tag grid has an invalid black border")
    if any(grid[row][1] != "1" or grid[row][8] != "1" for row in range(1, 9)):
        raise ValueError("official tag grid has an invalid black border")


def _grid_image(grid: tuple[str, ...]) -> Image.Image:
    _validate_grid(grid)
    small = np.array(
        [[0 if value == "1" else 255 for value in row] for row in grid],
        dtype=np.uint8,
    )
    image = Image.fromarray(small, mode="L")
    side_pixels = len(grid) * PNG_PIXELS_PER_CELL
    return image.resize((side_pixels, side_pixels), resample=Image.Resampling.NEAREST)


def _verify_detection(image: Image.Image, expected_id: int) -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    pixels = np.asarray(image)
    padded = cv2.copyMakeBorder(
        pixels, 200, 200, 200, 200, cv2.BORDER_CONSTANT, value=255
    )
    _corners, ids, _rejected = detector.detectMarkers(padded)
    detected = [] if ids is None else ids.ravel().tolist()
    if detected != [expected_id]:
        raise RuntimeError(
            f"generated {FAMILY} ID {expected_id} decoded as {detected}"
        )


def _write_svg(path: Path, label: str, tag_id: int, grid: tuple[str, ...]) -> None:
    _validate_grid(grid)
    height_mm = 62.0
    rects = []
    for row, values in enumerate(grid):
        for column, value in enumerate(values):
            if value == "1":
                rects.append(
                    f'<rect x="{column * CELL_SIZE_MM:.3f}" '
                    f'y="{row * CELL_SIZE_MM:.3f}" '
                    f'width="{CELL_SIZE_MM:.3f}" height="{CELL_SIZE_MM:.3f}"/>'
                )
    svg = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{FULL_GRID_SIZE_MM:.3f}mm" '
        f'height="{height_mm:.3f}mm" viewBox="0 0 {FULL_GRID_SIZE_MM:.3f} {height_mm:.3f}">\n'
        f'<rect width="{FULL_GRID_SIZE_MM:.3f}" height="{height_mm:.3f}" fill="white"/>\n'
        '<g fill="black" shape-rendering="crispEdges">\n'
        + "\n".join(rects)
        + "\n</g>\n"
        f'<text x="{FULL_GRID_SIZE_MM / 2:.3f}" y="57" text-anchor="middle" '
        'font-family="DejaVu Sans, sans-serif" font-size="4" font-weight="bold">'
        f'{label.upper()} UMI  |  {FAMILY} ID {tag_id}</text>\n'
        '</svg>\n'
    )
    path.write_text(svg, encoding="utf-8")


def _draw_pdf_tag(
    pdf: canvas.Canvas,
    *,
    x_mm: float,
    y_mm: float,
    label: str,
    tag_id: int,
    grid: tuple[str, ...],
) -> None:
    _validate_grid(grid)
    x = x_mm * MM_TO_PT
    y = y_mm * MM_TO_PT
    cell = CELL_SIZE_MM * MM_TO_PT
    pdf.setFillColorRGB(1, 1, 1)
    pdf.rect(x, y, FULL_GRID_SIZE_MM * MM_TO_PT, FULL_GRID_SIZE_MM * MM_TO_PT, fill=1, stroke=0)
    pdf.setFillColorRGB(0, 0, 0)
    for row, values in enumerate(grid):
        for column, value in enumerate(values):
            if value == "1":
                pdf.rect(
                    x + column * cell,
                    y + (9 - row) * cell,
                    cell,
                    cell,
                    fill=1,
                    stroke=0,
                )
    pdf.setStrokeColorRGB(0.65, 0.65, 0.65)
    pdf.setLineWidth(0.2 * MM_TO_PT)
    pdf.rect(
        x - 2 * MM_TO_PT,
        y - 10 * MM_TO_PT,
        54 * MM_TO_PT,
        62 * MM_TO_PT,
        fill=0,
        stroke=1,
    )
    pdf.setFillColorRGB(0, 0, 0)
    pdf.setFont("Helvetica-Bold", 11)
    pdf.drawCentredString(
        x + 25 * MM_TO_PT,
        y - 6 * MM_TO_PT,
        f"{label.upper()} UMI  |  {FAMILY} ID {tag_id}",
    )


def _write_pdf(path: Path) -> None:
    pdf = canvas.Canvas(str(path), pagesize=A4, pageCompression=0)
    page_width, page_height = A4
    pdf.setTitle("UMI LEFT RIGHT AprilTag 36h11 - 40 mm")
    pdf.setFont("Helvetica-Bold", 15)
    pdf.drawCentredString(
        page_width / 2,
        page_height - 20 * MM_TO_PT,
        "UMI spatial alignment tags",
    )
    pdf.setFont("Helvetica", 10)
    pdf.drawCentredString(
        page_width / 2,
        page_height - 27 * MM_TO_PT,
        "Print at 100% / Actual size. Black tag square = 40.0 mm; full white grid = 50.0 mm.",
    )
    _draw_pdf_tag(
        pdf,
        x_mm=42,
        y_mm=165,
        label="left",
        tag_id=TAGS["left"]["id"],
        grid=TAGS["left"]["grid"],
    )
    _draw_pdf_tag(
        pdf,
        x_mm=118,
        y_mm=165,
        label="right",
        tag_id=TAGS["right"]["id"],
        grid=TAGS["right"]["grid"],
    )

    ruler_x = 55 * MM_TO_PT
    ruler_y = 55 * MM_TO_PT
    pdf.setStrokeColorRGB(0, 0, 0)
    pdf.setLineWidth(0.35 * MM_TO_PT)
    pdf.line(ruler_x, ruler_y, ruler_x + 100 * MM_TO_PT, ruler_y)
    pdf.setFont("Helvetica", 8)
    for index in range(11):
        x = ruler_x + index * 10 * MM_TO_PT
        tick = 4 if index in (0, 10) else 2.5
        pdf.line(x, ruler_y - tick * MM_TO_PT, x, ruler_y + tick * MM_TO_PT)
        pdf.drawCentredString(x, ruler_y - 7 * MM_TO_PT, str(index * 10))
    pdf.drawCentredString(
        ruler_x + 50 * MM_TO_PT,
        ruler_y + 7 * MM_TO_PT,
        "100 mm verification ruler",
    )
    pdf.setFont("Helvetica", 8)
    pdf.drawCentredString(
        page_width / 2,
        28 * MM_TO_PT,
        "Measure the black square and ruler after printing. Do not use Fit to page.",
    )
    pdf.showPage()
    pdf.save()


def generate(output_dir: Path) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    files = []
    for label, spec in TAGS.items():
        tag_id = spec["id"]
        grid = spec["grid"]
        image = _grid_image(grid)
        _verify_detection(image, tag_id)
        stem = f"umi_{label}_{FAMILY}_id{tag_id}_40mm"
        png_path = output_dir / f"{stem}.png"
        image.save(png_path, dpi=(PNG_DPI, PNG_DPI), optimize=False)
        svg_path = output_dir / f"{stem}.svg"
        _write_svg(svg_path, label, tag_id, grid)
        files.extend((png_path.name, svg_path.name))

    pdf_path = output_dir / "umi_left_right_tag36h11_40mm_a4.pdf"
    _write_pdf(pdf_path)
    files.append(pdf_path.name)
    manifest = {
        "schema": "three-device-slam.apriltag-print-assets.v1",
        "family": FAMILY,
        "official_source": "https://github.com/AprilRobotics/apriltag-imgs",
        "official_source_commit": OFFICIAL_SOURCE_COMMIT,
        "effective_black_square_mm": EFFECTIVE_TAG_SIZE_MM,
        "full_grid_with_quiet_zone_mm": FULL_GRID_SIZE_MM,
        "tags": {
            label: {
                "id": spec["id"],
                "official_png_sha256": spec["official_png_sha256"],
            }
            for label, spec in TAGS.items()
        },
        "files": files,
        "verification": "OpenCV DICT_APRILTAG_36h11 decoded each generated PNG to its expected ID",
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("assets/calibration_tags"),
    )
    args = parser.parse_args()
    print(json.dumps(generate(args.output_dir), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
