#!/usr/bin/env python3
"""Generate the printable chessboard calibration target used by calibrate.py.

PDF, and pure stdlib. Both of those are deliberate:

  * PDF carries an explicit page size in points, so "Actual size" in a print
    dialog reproduces the squares at their stated mm. SVG and PNG both leave the
    final scale up to whatever renders them, and a board that prints at the wrong
    size is the WORST failure mode available here -- calibrate.py multiplies the
    operator's square_mm straight into board_object_points(), so a 4% scale error
    is a silent 4% error on every depth reading, with nothing in the solve or the
    epipolar RMS looking the least bit wrong.
  * no imaging library, so this runs on the device or any dev box as-is.

Geometry is driven by calibrate.py's own defaults, so the printed board and the
app's factory board settings cannot drift apart.

    python3 make_target.py            # writes the A4 + Letter PDFs
"""

import os
import sys
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

# Inner corners, per calibrate.DEFAULT_BOARD_COLS / _ROWS. Imported rather than
# duplicated, but with a literal fallback so this stays runnable standalone (and
# on a box without cv2, which calibrate.py imports).
try:
    from calibrate import DEFAULT_BOARD_COLS, DEFAULT_BOARD_ROWS
except Exception:
    DEFAULT_BOARD_COLS, DEFAULT_BOARD_ROWS = 9, 6

# A board of N x M SQUARES has (N-1) x (M-1) inner corners.
COLS_SQUARES = DEFAULT_BOARD_COLS + 1
ROWS_SQUARES = DEFAULT_BOARD_ROWS + 1

# 20 mm rather than calibrate.DEFAULT_SQUARE_MM's 25: at 25 mm a 10x7 board is
# 250x175 mm, which leaves under one square of white margin on A4/Letter. The
# classic detector needs a quiet zone wider than that to close its boundary
# quads, and the margin matters more than matching the default -- the operator
# types the square size in anyway.
SQUARE_MM = 20.0

# White space between the board and anything else on the page. One full square is
# the practical minimum for findChessboardCorners.
MIN_QUIET_MM = SQUARE_MM

PT_PER_MM = 72.0 / 25.4

PAGES = {
    "a4":     (297.0, 210.0),   # landscape
    "letter": (279.4, 215.9),   # landscape
}


def _esc(text):
    for old, new in (("\\", r"\\"), ("(", r"\("), (")", r"\)")):
        text = text.replace(old, new)
    return text


class _Pdf:
    """Minimal single-page PDF writer (vector rectangles, lines and text)."""

    def __init__(self, width_mm, height_mm):
        self.width = width_mm * PT_PER_MM
        self.height = height_mm * PT_PER_MM
        self.ops = []

    # PDF's origin is BOTTOM-left; every helper here takes top-left mm and flips,
    # so the layout code reads the same way the page does.
    def _y(self, top_mm, height_mm=0.0):
        return self.height - (top_mm + height_mm) * PT_PER_MM

    def rect(self, x_mm, top_mm, w_mm, h_mm, gray=0.0):
        self.ops.append("%.4f g %.4f %.4f %.4f %.4f re f"
                        % (gray, x_mm * PT_PER_MM, self._y(top_mm, h_mm),
                           w_mm * PT_PER_MM, h_mm * PT_PER_MM))

    def line(self, x1_mm, top1_mm, x2_mm, top2_mm, width_mm=0.25):
        self.ops.append("0 G %.4f w %.4f %.4f m %.4f %.4f l S"
                        % (width_mm * PT_PER_MM,
                           x1_mm * PT_PER_MM, self._y(top1_mm),
                           x2_mm * PT_PER_MM, self._y(top2_mm)))

    def text(self, x_mm, top_mm, size_pt, body):
        self.ops.append("BT 0 g /F1 %.2f Tf %.4f %.4f Td (%s) Tj ET"
                        % (size_pt, x_mm * PT_PER_MM, self._y(top_mm), _esc(body)))

    def write(self, path):
        stream = zlib.compress(("\n".join(self.ops)).encode("ascii"))
        objects = [
            b"<< /Type /Catalog /Pages 2 0 R >>",
            b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
            ("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.4f %.4f] "
             "/Contents 4 0 R /Resources << /Font << /F1 5 0 R >> >> >>"
             % (self.width, self.height)).encode("ascii"),
            (b"<< /Length %d /Filter /FlateDecode >>\nstream\n" % len(stream)) +
            stream + b"\nendstream",
            b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
        ]

        out = bytearray(b"%PDF-1.4\n")
        offsets = []
        for index, body in enumerate(objects, start=1):
            offsets.append(len(out))
            out += b"%d 0 obj\n" % index + body + b"\nendobj\n"

        xref_at = len(out)
        out += b"xref\n0 %d\n" % (len(objects) + 1)
        out += b"0000000000 65535 f \n"
        for offset in offsets:
            out += b"%010d 00000 n \n" % offset
        out += (b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n"
                % (len(objects) + 1, xref_at))

        with open(path, "wb") as handle:
            handle.write(bytes(out))
        return len(out)


def build(page_w, page_h, cols=COLS_SQUARES, rows=ROWS_SQUARES, square=SQUARE_MM):
    board_w, board_h = cols * square, rows * square

    # The board sits high on the page: the scale ruler and caption go underneath,
    # and they must clear the quiet zone rather than eat into it.
    top = MIN_QUIET_MM + 2.0
    left = (page_w - board_w) / 2.0
    if left < MIN_QUIET_MM or top + board_h + MIN_QUIET_MM > page_h:
        raise ValueError("board %gx%g mm leaves too little quiet zone on %gx%g mm"
                         % (board_w, board_h, page_w, page_h))

    pdf = _Pdf(page_w, page_h)
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                pdf.rect(left + c * square, top + r * square, square, square)

    # A ruler beats an instruction. "Print at 100%" is ignored or overridden all
    # the time, and the operator has no other way to catch it -- so give them
    # something to physically measure. If this reads 100 mm, square_mm is right.
    ruler_top = top + board_h + MIN_QUIET_MM + 6.0
    pdf.line(left, ruler_top, left + 100.0, ruler_top, width_mm=0.3)
    for tick in range(0, 101, 10):
        length = 3.5 if tick % 50 == 0 else 2.0
        pdf.line(left + tick, ruler_top - length, left + tick, ruler_top, width_mm=0.3)
    pdf.text(left + 102.0, ruler_top, 8.0,
             "<- this must measure exactly 100 mm. If it does not, reprint at "
             "100% / Actual Size / Fit to printable area.")

    pdf.text(left, ruler_top + 8.0, 9.0,
             "%d x %d squares  =  %d x %d INNER CORNERS  |  %g mm squares"
             % (cols, rows, cols - 1, rows - 1, square))
    pdf.text(left, ruler_top + 14.0, 8.0,
             "Stereo Calibration: set Board Corner Columns=%d, "
             "Rows=%d, Square Size=%g mm." % (cols - 1, rows - 1, square))
    pdf.text(left, ruler_top + 19.0, 8.0,
             "Mount FLAT and RIGID. Do not trim the white border -- the corner "
             "detector needs it.")
    return pdf


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    for name, (w, h) in PAGES.items():
        path = os.path.join(here, "chessboard_%dx%d_%gmm_%s.pdf"
                            % (COLS_SQUARES - 1, ROWS_SQUARES - 1, SQUARE_MM, name))
        size = build(w, h).write(path)
        print("%s  (%d bytes, board %gx%g mm on %gx%g mm)"
              % (os.path.basename(path), size,
                 COLS_SQUARES * SQUARE_MM, ROWS_SQUARES * SQUARE_MM, w, h))
