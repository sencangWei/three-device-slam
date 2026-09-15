"""Generate one 40/80mm tag36h11 on A4, with 10mm white margins (default ID2)."""
import argparse
from pathlib import Path
import hashlib
import json

import cv2
import numpy as np
from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--id', type=int, default=2)
    parser.add_argument('--size-mm', type=int, choices=(40, 80), default=40)
    parser.add_argument('--output', type=Path, default=Path("assets/calibration_tags/external_id2_20260905"))
    args = parser.parse_args()
    output = args.output
    if output.exists():
        raise FileExistsError(output)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    if not 0 <= args.id < len(dictionary.bytesList):
        parser.error('id outside tag36h11 dictionary')
    bits = cv2.aruco.generateImageMarker(dictionary, args.id, 8, borderBits=1)
    black_pixels = np.repeat(np.repeat(bits, 100, axis=0), 100, axis=1)
    pixels = np.pad(black_pixels, round(800 * 10 / args.size_mm), constant_values=255)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    _, ids, _ = detector.detectMarkers(pixels)
    if ids is None or ids.ravel().tolist() != [args.id]:
        raise RuntimeError(f"Generated tag must decode as ID{args.id}")
    output.mkdir(parents=True)
    png = output / f"external_tag36h11_id{args.id}_{args.size_mm}mm.png"
    dpi = 800 * 25.4 / args.size_mm
    Image.fromarray(pixels).save(png, dpi=(dpi, dpi))
    pdf_path = output / f"external_tag36h11_id{args.id}_{args.size_mm}mm_a4.pdf"
    pdf = canvas.Canvas(str(pdf_path), pagesize=A4)
    mm = 72 / 25.4
    pdf.setTitle(f"External AprilTag 36h11 ID{args.id} - {args.size_mm}mm")
    pdf.setFont("Helvetica-Bold", 16)
    pdf.drawCentredString(A4[0] / 2, 265 * mm, f"EXTERNAL TAG | tag36h11 | ID {args.id}")
    pdf.setFont("Helvetica", 11)
    pdf.drawCentredString(A4[0] / 2, 252 * mm, f"Actual size / 100%. Black square: {args.size_mm:.1f} mm.")
    left = 105 - args.size_mm / 2
    bottom = 190 - args.size_mm / 2
    cell = args.size_mm / 8
    for row in range(8):
        for col in range(8):
            if bits[row, col] == 0:
                pdf.rect((left + col * cell) * mm, (bottom + (7 - row) * cell) * mm, cell * mm, cell * mm, fill=1, stroke=0)
    pdf.setDash(2, 3)
    pdf.setStrokeColorRGB(.7, .7, .7)
    pdf.rect((left - 10) * mm, (bottom - 10) * mm, (args.size_mm + 20) * mm, (args.size_mm + 20) * mm, fill=0, stroke=1)
    pdf.setDash()
    pdf.setStrokeColorRGB(0, 0, 0)
    pdf.drawCentredString(A4[0] / 2, (bottom - 21) * mm, "Keep the 10 mm white margin on EVERY side.")
    pdf.line(55 * mm, 80 * mm, 155 * mm, 80 * mm)
    for i in range(11):
        x = (55 + i * 10) * mm
        pdf.line(x, 77 * mm, x, 83 * mm)
    pdf.drawCentredString(A4[0] / 2, 68 * mm, "Verification ruler: 100 mm end to end")
    pdf.showPage()
    pdf.save()
    manifest = {
        "family": "tag36h11", "id": args.id, "black_square_mm": args.size_mm,
        "white_margin_mm": 10, "opencv_version": cv2.__version__,
        "pattern_source": "OpenCV DICT_APRILTAG_36H11.generateImageMarker",
        "digital_decode": [args.id], "physical_measurement": "PENDING",
        "files": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (png, pdf_path)},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
