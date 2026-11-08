#!/usr/bin/env python3
"""
see_gui.py - CLI vision bridge.

Why this exists
---------------
VS Code's built-in image viewer returns image data inside a tool-result block.
The upstream API requires `messages[*].tool.content` to be a plain string, so
returning an image there aborts the whole chat turn with:

    Input should be a valid string ... messages.N.tool.content.str

This script sidesteps that entirely: it sends the picture as a normal
`role: "user"` message to a vision model and prints back *text*. The agent
reads it through the terminal tool, which is always a string.

Usage
-----
    python3 see_gui.py <image_path> [prompt]

    python3 see_gui.py /tmp/now.jpg "Is the head level? Any tilt?"
    python3 see_gui.py shot.png                 # default inspection prompt
    python3 see_gui.py --check                  # verify API key + model work
    python3 see_gui.py --crop 0,0,640,480 img.jpg "left half only"

Environment
-----------
    DEEPINFRA_API_KEY   required (also read from ./.env or ~/.env)
    SEE_MODEL           optional, default deepseek-ai/DeepSeek-V4.1-Flash-1017
    SEE_MAX_DIM         optional, default 1280
    SEE_API_URL         optional, override endpoint
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import os
import sys

API_URL_DEFAULT = "https://api.deepinfra.com/v1/openai/chat/completions"
MODEL_DEFAULT = "deepseek-ai/DeepSeek-V4.1-Flash-1017"
# 1280 keeps GUI text legible while staying well under vision token limits.
MAX_DIM_DEFAULT = 1280

DEFAULT_PROMPT = (
    "Inspect this image (it may be a GUI screenshot, a camera frame, or a photo). "
    "Report concretely: what is visible, the layout/arrangement, any text you can read "
    "(quote it verbatim), alignment or rendering problems, and anything that looks like "
    "a bug or a visual defect. Be specific and avoid generic filler."
)


def load_env() -> None:
    """Pull DEEPINFRA_API_KEY from a .env file if it is not already exported."""
    if os.environ.get("DEEPINFRA_API_KEY"):
        return
    try:
        from dotenv import load_dotenv  # type: ignore
    except ImportError:
        return
    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in (
        os.path.join(here, ".env"),
        os.path.join(os.path.expanduser("~"), ".env"),
        ".env",
    ):
        if os.path.isfile(candidate):
            load_dotenv(candidate, override=False)
            if os.environ.get("DEEPINFRA_API_KEY"):
                return


def encode_image(path: str, max_dim: int, crop: str | None = None) -> tuple[str, str]:
    """Return (base64_jpeg, human_readable_geometry_note)."""
    try:
        from PIL import Image  # type: ignore
    except ImportError:
        sys.exit("ERROR: Pillow is required. Install with: pip install pillow")

    with Image.open(path) as img:
        img = img.convert("RGB")
        orig = img.size

        if crop:
            try:
                x, y, w, h = (int(v) for v in crop.split(","))
                img = img.crop((x, y, x + w, y + h))
            except Exception:
                sys.exit(f"ERROR: --crop wants x,y,w,h as integers, got {crop!r}")

        if max_dim > 0 and max(img.size) > max_dim:
            img.thumbnail((max_dim, max_dim), Image.Resampling.LANCZOS)

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=85)
        b64 = base64.b64encode(buf.getvalue()).decode("utf-8")

    note = f"{orig[0]}x{orig[1]} original"
    if img.size != orig:
        note += f" -> sent as {img.size[0]}x{img.size[1]}"
    return b64, note


def call_api(api_key: str, model: str, api_url: str, prompt: str, b64: str | None) -> str:
    try:
        import requests  # type: ignore
    except ImportError:
        sys.exit("ERROR: requests is required. Install with: pip install requests")

    if b64 is None:
        content: object = prompt
    else:
        content = [
            {"type": "text", "text": prompt},
            {
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{b64}"},
            },
        ]

    payload = {
        "model": model,
        "messages": [{"role": "user", "content": content}],
        "max_tokens": 1024,
        "temperature": 0.1,
    }

    try:
        resp = requests.post(
            api_url,
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
            json=payload,
            timeout=90,
        )
    except Exception as exc:  # noqa: BLE001 - network errors are all fatal here
        sys.exit(f"ERROR: request failed: {exc.__class__.__name__}: {exc}")

    if resp.status_code != 200:
        sys.exit(f"ERROR: API returned {resp.status_code}: {resp.text[:800]}")

    try:
        data = resp.json()
        return data["choices"][0]["message"]["content"]
    except (ValueError, KeyError, IndexError):
        sys.exit(f"ERROR: unexpected response shape: {resp.text[:800]}")


def main() -> None:
    # Model output routinely contains characters outside latin-1 (arrows,
    # arrows in GUI labels, CJK). On a terminal whose locale resolves to
    # latin-1, print() raises UnicodeEncodeError *after* the API call has
    # already been paid for, losing the whole answer. Force UTF-8 and degrade
    # unencodable characters to '?' rather than crashing.
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except (AttributeError, ValueError):
        pass

    ap = argparse.ArgumentParser(
        description="Send an image to a vision model and print text back.",
    )
    ap.add_argument("image", nargs="?", help="path to .jpg/.png/.webp")
    ap.add_argument("prompt", nargs="?", default=None, help="question about the image")
    ap.add_argument("--check", action="store_true", help="verify key/model without an image")
    ap.add_argument("--crop", default=None, metavar="x,y,w,h", help="crop before sending")
    ap.add_argument("--max-dim", type=int, default=None, help="override SEE_MAX_DIM")
    ap.add_argument("--model", default=None, help="override SEE_MODEL")
    ap.add_argument("--json", action="store_true", help="print raw JSON instead of text")
    args = ap.parse_args()

    load_env()
    api_key = os.environ.get("DEEPINFRA_API_KEY", "").strip()
    if not api_key:
        sys.exit(
            "ERROR: DEEPINFRA_API_KEY is not set.\n"
            "  Add it once, without exposing it in shell history:\n"
            "    read -rsp 'DeepInfra key: ' K && echo \"DEEPINFRA_API_KEY=$K\" >> ~/.env\n"
            "  Then this script picks it up automatically."
        )

    model = args.model or os.environ.get("SEE_MODEL") or MODEL_DEFAULT
    api_url = os.environ.get("SEE_API_URL") or API_URL_DEFAULT
    max_dim = args.max_dim if args.max_dim is not None else int(
        os.environ.get("SEE_MAX_DIM", MAX_DIM_DEFAULT)
    )

    if args.check:
        text = call_api(api_key, model, api_url, "Reply with exactly: VISION_OK", None)
        print(f"model : {model}")
        print(f"reply : {text.strip()[:200]}")
        sys.exit(0 if "VISION_OK" in text else 1)

    if not args.image:
        ap.error("image path is required (or use --check)")

    if not os.path.isfile(args.image):
        sys.exit(f"ERROR: image not found: {args.image}")

    b64, note = encode_image(args.image, max_dim, args.crop)
    prompt = args.prompt or DEFAULT_PROMPT
    text = call_api(api_key, model, api_url, prompt, b64)

    if args.json:
        print(json.dumps({"image": args.image, "geometry": note, "text": text}, indent=2))
    else:
        print(f"[see_gui] {os.path.basename(args.image)} | {note} | model={model}")
        print("-" * 68)
        print(text.strip())


if __name__ == "__main__":
    main()
