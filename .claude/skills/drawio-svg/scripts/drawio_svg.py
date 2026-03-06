#!/usr/bin/env python3
"""DrawIO SVG build / validate / extract tool.

Eliminates manual HTML-encoding errors and viewBox miscalculations
when creating or editing .drawio.svg files.
"""

import argparse
import html
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def html_encode_for_attr(text: str) -> str:
    """Encode raw XML for embedding in an SVG content attribute."""
    text = text.replace("&", "&amp;")
    text = text.replace("<", "&lt;")
    text = text.replace(">", "&gt;")
    text = text.replace('"', "&quot;")
    return text


def html_decode_attr(text: str) -> str:
    """Decode HTML entities from an SVG content attribute."""
    return html.unescape(text)


def _find_attr(tag_str: str, attr: str):
    """Extract a numeric attribute value from an SVG element string."""
    m = re.search(rf'{attr}\s*=\s*["\']([^"\']*)["\']', tag_str)
    if m:
        try:
            return float(m.group(1))
        except ValueError:
            return None
    return None


def calculate_viewbox(svg_body: str) -> tuple:
    """Return (min_x, min_y, width, height) covering all SVG elements."""
    min_x = float("inf")
    min_y = float("inf")
    max_x = float("-inf")
    max_y = float("-inf")

    def expand(x, y):
        nonlocal min_x, min_y, max_x, max_y
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        max_x = max(max_x, x)
        max_y = max(max_y, y)

    # <rect>, <image>, <foreignObject> with x/y/width/height
    for m in re.finditer(r"<(?:rect|image|foreignObject)\b[^>]*?>", svg_body):
        tag = m.group(0)
        x = _find_attr(tag, "x")
        y = _find_attr(tag, "y")
        w = _find_attr(tag, "width")
        h = _find_attr(tag, "height")
        if x is not None and y is not None:
            expand(x, y)
            if w is not None and h is not None:
                expand(x + w, y + h)

    # <text> with x/y
    for m in re.finditer(r"<text\b[^>]*?>", svg_body):
        tag = m.group(0)
        x = _find_attr(tag, "x")
        y = _find_attr(tag, "y")
        if x is not None and y is not None:
            expand(x, y)
            # text extends to the right/below; rough padding
            expand(x + 10, y + 5)

    # <line> with x1/y1/x2/y2
    for m in re.finditer(r"<line\b[^>]*?>", svg_body):
        tag = m.group(0)
        for pair in [("x1", "y1"), ("x2", "y2")]:
            x = _find_attr(tag, pair[0])
            y = _find_attr(tag, pair[1])
            if x is not None and y is not None:
                expand(x, y)

    # <path d="..."> – extract M and L coordinates
    for m in re.finditer(r"<path\b[^>]*?>", svg_body):
        tag = m.group(0)
        d_match = re.search(r'd\s*=\s*"([^"]*)"', tag)
        if d_match:
            for coord in re.finditer(r"[ML]\s*([\d.]+)\s+([\d.]+)", d_match.group(1)):
                expand(float(coord.group(1)), float(coord.group(2)))

    # <circle> with cx/cy/r
    for m in re.finditer(r"<circle\b[^>]*?>", svg_body):
        tag = m.group(0)
        cx = _find_attr(tag, "cx")
        cy = _find_attr(tag, "cy")
        r = _find_attr(tag, "r")
        if cx is not None and cy is not None and r is not None:
            expand(cx - r, cy - r)
            expand(cx + r, cy + r)

    # <ellipse> with cx/cy/rx/ry
    for m in re.finditer(r"<ellipse\b[^>]*?>", svg_body):
        tag = m.group(0)
        cx = _find_attr(tag, "cx")
        cy = _find_attr(tag, "cy")
        rx = _find_attr(tag, "rx")
        ry = _find_attr(tag, "ry")
        if None not in (cx, cy, rx, ry):
            expand(cx - rx, cy - ry)
            expand(cx + rx, cy + ry)

    if min_x == float("inf"):
        return (0, 0, 800, 600)

    padding = 10
    return (
        min_x - padding,
        min_y - padding,
        max_x - min_x + 2 * padding,
        max_y - min_y + 2 * padding,
    )


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

DEFAULT_DEFS = """\
    <defs>
        <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="10" refY="3.5" orient="auto">
            <polygon points="0 0, 10 3.5, 0 7" fill="#555"/>
        </marker>
    </defs>"""


def _extract_and_merge_defs(svg_body: str, custom_defs: str | None) -> tuple:
    """Extract <defs> from SVG body, merge with custom/default defs.

    Returns (cleaned_svg_body, merged_defs_block).
    """
    body_defs_contents = []

    def collect_defs(match):
        body_defs_contents.append(match.group(1))
        return ""

    cleaned_body = re.sub(
        r"<defs\b[^>]*>(.*?)</defs>",
        collect_defs,
        svg_body,
        flags=re.DOTALL,
    )

    base_defs = custom_defs if custom_defs else DEFAULT_DEFS

    if body_defs_contents:
        base_inner_match = re.search(
            r"<defs[^>]*>(.*?)</defs>", base_defs, re.DOTALL
        )
        base_inner = base_inner_match.group(1) if base_inner_match else ""
        existing_ids = set(re.findall(r'id="([^"]*)"', base_inner))

        extra_markers = []
        for body_def_content in body_defs_contents:
            for marker_match in re.finditer(
                r'(<marker\b[^>]*id="([^"]*)"[^>]*>.*?</marker>)',
                body_def_content,
                re.DOTALL,
            ):
                marker_id = marker_match.group(2)
                if marker_id not in existing_ids:
                    existing_ids.add(marker_id)
                    extra_markers.append(marker_match.group(1))

        if extra_markers:
            merged_inner = base_inner.rstrip() + "\n" + "\n".join(
                f"        {m}" for m in extra_markers
            )
            base_defs = f"    <defs>{merged_inner}\n    </defs>"

    return cleaned_body.strip(), base_defs


def cmd_build(args):
    """Build a .drawio.svg from raw mxfile XML and SVG body elements."""
    mxfile_xml = Path(args.mxfile).read_text(encoding="utf-8").strip()
    svg_body = Path(args.svg_body).read_text(encoding="utf-8").strip()

    # Validate the mxfile XML parses correctly
    try:
        ET.fromstring(mxfile_xml)
    except ET.ParseError as e:
        print(f"ERROR: mxfile XML is not valid: {e}", file=sys.stderr)
        sys.exit(1)

    # Read custom defs if provided
    custom_defs = None
    if args.defs:
        custom_defs = Path(args.defs).read_text(encoding="utf-8").strip()

    # Extract defs from SVG body and merge (prevents duplication)
    svg_body, defs = _extract_and_merge_defs(svg_body, custom_defs)

    encoded = html_encode_for_attr(mxfile_xml)

    vb = calculate_viewbox(svg_body)
    viewbox = f"{vb[0]:.1f} {vb[1]:.1f} {vb[2]:.1f} {vb[3]:.1f}"
    width = f"{vb[2]:.0f}px"
    height = f"{vb[3]:.0f}px"

    svg = (
        f'<svg host="65bd71144e" xmlns="http://www.w3.org/2000/svg"'
        f' xmlns:xlink="http://www.w3.org/1999/xlink" version="1.1"'
        f' width="{width}" height="{height}" viewBox="{viewbox}"'
        f' style="background: transparent;"'
        f' content="{encoded}">\n'
        f"{defs}\n"
        f"    <g>\n"
        f"{svg_body}\n"
        f"    </g>\n"
        f"</svg>\n"
    )

    Path(args.output).write_text(svg, encoding="utf-8")
    print(f"OK: {args.output}  ({width} x {height},  viewBox={viewbox})")


def cmd_validate(args):
    """Validate a .drawio.svg file."""
    path = Path(args.file)
    content = path.read_text(encoding="utf-8")
    errors = []
    warnings = []

    # 1. Must start with <svg
    if not content.strip().startswith("<svg"):
        errors.append("File does not start with <svg> tag")

    # 2. content attribute must exist and decode to valid mxfile XML
    m = re.search(r'content="([^"]*)"', content)
    if not m:
        errors.append("No content attribute found in <svg> tag")
    else:
        encoded = m.group(1)
        decoded = html_decode_attr(encoded)

        if not decoded.strip().startswith("<mxfile"):
            errors.append(
                f"Decoded content does not start with <mxfile>. "
                f"Starts with: {decoded[:80]!r}"
            )

        try:
            root = ET.fromstring(decoded)
            # Check basic structure
            diagram = root.find("diagram")
            if diagram is None:
                warnings.append("mxfile has no <diagram> child")
            else:
                model = diagram.find("mxGraphModel")
                if model is None:
                    warnings.append("diagram has no <mxGraphModel> child")
        except ET.ParseError as e:
            errors.append(f"Decoded mxfile XML is not valid: {e}")

    # 3. viewBox present
    if 'viewBox="' not in content:
        warnings.append("No viewBox attribute found")

    # 4. SVG drawing elements exist
    has_drawing = any(
        f"<{tag}" in content
        for tag in ("rect", "text", "line", "path", "circle", "ellipse")
    )
    if not has_drawing:
        warnings.append("No SVG drawing elements found")

    # 5. Check mxCell count vs SVG element count
    if m:
        decoded = html_decode_attr(m.group(1))
        vertex_count = len(re.findall(r'vertex="1"', decoded))
        edge_count = len(re.findall(r'edge="1"', decoded))
        rect_count = len(re.findall(r"<rect\b", content))
        line_path_count = len(re.findall(r"<(?:line|path)\b", content))

        if vertex_count > 0 and rect_count == 0:
            warnings.append(
                f"{vertex_count} mxCell vertices but 0 SVG rects — "
                f"SVG layer may be missing"
            )
        if edge_count > 0 and line_path_count == 0:
            warnings.append(
                f"{edge_count} mxCell edges but 0 SVG lines/paths — "
                f"SVG layer may be missing"
            )

        # Reverse check: SVG arrows without mxCell edges
        arrow_count = len(re.findall(r'marker-end\s*=\s*["\']', content))
        if arrow_count > 0 and edge_count == 0:
            errors.append(
                f"{arrow_count} SVG arrows (marker-end) but 0 mxCell edges — "
                f"arrows will be invisible in DrawIO editor. "
                f"Add edge=\"1\" mxCell elements for each connection."
            )
        elif arrow_count > 0 and edge_count > 0 and arrow_count > edge_count * 2:
            warnings.append(
                f"{arrow_count} SVG arrows but only {edge_count} mxCell edges — "
                f"some arrows may be missing from mxfile XML"
            )

    # Report
    ok = True
    if errors:
        ok = False
        print(f"ERRORS ({path}):")
        for e in errors:
            print(f"  x {e}")
    if warnings:
        print(f"WARNINGS ({path}):")
        for w in warnings:
            print(f"  ! {w}")
    if ok and not warnings:
        print(f"OK: {path}")

    sys.exit(0 if ok else 1)


def cmd_extract(args):
    """Extract raw mxfile XML from a .drawio.svg file."""
    content = Path(args.file).read_text(encoding="utf-8")
    m = re.search(r'content="([^"]*)"', content)
    if not m:
        print("ERROR: No content attribute found", file=sys.stderr)
        sys.exit(1)

    decoded = html_decode_attr(m.group(1))

    # Pretty-print if possible
    try:
        root = ET.fromstring(decoded)
        ET.indent(root)
        decoded = ET.tostring(root, encoding="unicode")
    except ET.ParseError:
        pass  # output as-is

    if args.output:
        Path(args.output).write_text(decoded, encoding="utf-8")
        print(f"Extracted to: {args.output}", file=sys.stderr)
    else:
        print(decoded)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(
        description="Build, validate, and extract DrawIO-compatible SVG files"
    )
    sub = p.add_subparsers(dest="command")

    # build
    bp = sub.add_parser("build", help="Build .drawio.svg from mxfile XML + SVG body")
    bp.add_argument("--mxfile", required=True, help="Path to raw mxfile XML file")
    bp.add_argument("--svg-body", required=True, help="Path to SVG body elements file")
    bp.add_argument("-o", "--output", required=True, help="Output .drawio.svg path")
    bp.add_argument("--defs", help="Path to custom <defs> block (optional)")

    # validate
    vp = sub.add_parser("validate", help="Validate a .drawio.svg file")
    vp.add_argument("file", help="Path to .drawio.svg file")

    # extract
    ep = sub.add_parser("extract", help="Extract raw mxfile XML from .drawio.svg")
    ep.add_argument("file", help="Path to .drawio.svg file")
    ep.add_argument("-o", "--output", help="Output file (default: stdout)")

    args = p.parse_args()
    if args.command == "build":
        cmd_build(args)
    elif args.command == "validate":
        cmd_validate(args)
    elif args.command == "extract":
        cmd_extract(args)
    else:
        p.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
