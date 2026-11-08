# Vision API — how to show an AI agent a picture

A practical guide to `tools/see_gui.py`, the workaround for VS Code's broken
image tool. Read this first in any new session where a screenshot, camera frame
or photo needs to be examined.

---

## 1. The constraint (read this before anything else)

**An AI agent in this workspace must never return an image inside a tool
result.** It does not work, and it does not fail gracefully.

VS Code's built-in image viewer puts image bytes into the `content` field of a
tool-result message. The upstream API requires that field to be a **plain
string**. When an image arrives there instead, the entire chat turn is aborted:

```
Input should be a valid string
type=invalid_request_error
param=messages.21.tool.content.str
```

The failure is total. The session does not recover — the poisoned message stays
in the conversation history and every subsequent turn fails the same way. There
is no partial output and no way to catch it in code. It is a schema violation on
the request, not a bug in the agent's logic.

This affects:

| Tool | Safe? | Why |
|---|---|---|
| `view_image` | ❌ **never** | Returns bytes in a tool result |
| `read_file` on `.png` `.jpg` | ❌ **never** | Same path |
| Terminal command printing text | ✅ | String, always valid |
| A **user-pasted** image | ✅ | Arrives as a normal user message |

That last row matters: if a person pastes a screenshot into chat, the agent can
reason about it directly. The restriction is only about images coming *back from
a tool*.

---

## 2. The workaround

`tools/see_gui.py` sends the picture as a normal `role: "user"` message to a
vision model and prints **text** to stdout. The agent reads that text through the
terminal tool, which is always a string. Nothing ever enters a tool result as an
image.

```
  image file ──► see_gui.py ──► vision model ──► text on stdout ──► agent
                                   (user role)
```

---

## 3. Setup (once per machine)

```bash
pip install pillow requests python-dotenv
```

Store the key **without putting it in shell history**:

```bash
read -rsp 'DeepInfra key: ' K && echo "DEEPINFRA_API_KEY=$K" >> ~/.env && unset K
```

The script looks for the key in this order:

1. `DEEPINFRA_API_KEY` in the environment
2. `./.env`
3. `~/.env`

Verify it works before relying on it:

```bash
python3 tools/see_gui.py --check
# model : deepseek-ai/DeepSeek-V4.1-Flash-1017
# reply : VISION_OK
```

`VISION_OK` means the key, the model name and the network path are all good. If
you see anything else, fix that before debugging anything visual — otherwise you
will misread an API failure as a GUI bug.

---

## 4. Usage

```bash
python3 tools/see_gui.py <image> "<question>"
```

### The question matters more than the image

A vague question wastes tokens and returns vague prose. Compare:

| ❌ Wasteful | ✅ Useful |
|---|---|
| "What do you see?" | "Read the battery voltage in the top right and say if it is below 11 V" |
| "Is this OK?" | "Is any label text clipped or overlapping another element?" |
| "Describe this" | "Which panel is the darkest — 4, 5, or 6 — and does it contain a drawing or just text?" |

Name the region, name the value you expect, and ask for a yes/no where possible.

### Options

| Flag | Use |
|---|---|
| `--check` | Verify key + model, no image |
| `--crop x,y,w,h` | Crop **before** downscaling (see below) |
| `--max-dim N` | Override the 1280 px limit |
| `--model NAME` | Override the model |
| `--json` | Machine-readable output |

### Cropping — the one trick worth remembering

Images larger than 1280 px are downscaled. Small text can vanish in that
downscale, and then the model reports "I cannot read it" rather than "the text
is missing" — which is easy to misdiagnose as a GUI bug.

Crop to the region of interest so it survives:

```bash
# 1320x830 screenshot, check the map panel in the bottom right
python3 tools/see_gui.py shot.png --crop 1000,400,320,300 \
    "Describe what is drawn in this panel: map, laser points, robot arrow?"
```

---

## 5. When to use it — and when not to

**Use it for genuinely visual questions:**

- Does the layout look right? Anything clipped, overlapping or misaligned?
- Is the camera feed live or showing a placeholder?
- What do the status values say?
- Is the map drawn with walls and a robot marker, or is it blank?

**Do not use it for things that can be measured.** Measuring is cheaper, exact,
and repeatable. Every defect in `robot/docs/report.md` §7 was found by
measurement, not by looking:

| Instead of asking "is the camera working?" | Measure it |
|---|---|
| Ask "does the servo move?" | Track frame displacement with ORB matching |
| Ask "is the map updating?" | `ros2 topic hz /map` on the *receiving* machine |
| Ask "did the layout break?" | Walk the widget tree, assert no zero-size widgets |
| Ask "is the text cut off?" | Compare `sizeHint()` against the actual geometry |

A useful pattern: **measure first, then confirm visually.** Dump the widget
geometry from a headless render, then take one screenshot and ask a narrow
question about it. If the two disagree, the disagreement is the bug.

---

## 6. Taking a screenshot to analyse

The operator VM may be headless from the agent's point of view, but Qt can render
offscreen and save a PNG. That is how the console screenshots were produced:

```python
import os
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
# ... construct the window, processEvents() for a few seconds ...
win.grab().save('/tmp/console.png', 'PNG')
```

Then copy it somewhere the agent can reach and ask a specific question:

```bash
python3 tools/see_gui.py /tmp/console.png "Is the camera panel showing an image?"
```

---

## 7. Checklist for a new session

1. `python3 tools/see_gui.py --check` → expect `VISION_OK`
2. If it fails, fix the key. Do not proceed with visual debugging.
3. Never call `view_image` or `read_file` on an image.
4. Crop before asking about small details.
5. Ask a specific question.
6. Prefer measuring over looking when the property is measurable.

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `DEEPINFRA_API_KEY is not set` | Key not in env or `~/.env` | Run the `read -rsp` setup above |
| `API returned 401` | Key revoked or wrong | Create a new key at deepinfra.com |
| `API returned 404` | Model name changed | `--model` with a current vision model |
| `unexpected response shape` | Model returned no text | Retry; check `--json` for the raw body |
| Model says "cannot read the text" | Detail lost in downscale | `--crop` to that region |
| `ERROR: image not found` | Wrong path | Use an absolute path |

> **Never paste the API key into a chat message.** It ends up in the transcript
> and in shell history. If a key has been exposed, rotate it at deepinfra.com and
> re-run the `read -rsp` setup.

---

## 9. Why not just fix the image tool?

Because the constraint is upstream of this workspace. The tool's return value is
serialised into a message field that the API defines as a string. Nothing an
agent, an extension or a prompt can do changes that — the request is rejected
before any model sees it.

The vision bridge is not a workaround for a bug that will be fixed; it is the
supported shape of the interaction. Treat it as the normal way to look at a
picture.
