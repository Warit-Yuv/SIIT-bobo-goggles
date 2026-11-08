# tools/

Development helpers. Nothing here is needed to operate the robot.

---

## `see_gui.py` — vision bridge for AI-assisted debugging

### The problem it solves

VS Code's built-in image viewer returns image data *inside a tool-result block*.
The upstream API requires `messages[*].tool.content` to be a plain string, so an
image returned that way aborts the whole chat turn:

```
Input should be a valid string, type=invalid_request_type,
param=messages.N.tool.content.str
```

This is not a limitation you can code around in the model — it is a schema
constraint on the request.

### What it does instead

Sends the picture as a normal `role: "user"` message to a vision model and
prints **text** back. Text in a tool result is always valid, so the session
never breaks.

```bash
python3 tools/see_gui.py screenshot.png "Is the E-STOP button red?"
python3 tools/see_gui.py --crop 0,0,640,480 frame.jpg "Read the servo value"
python3 tools/see_gui.py --check          # verify the key and model work
```

### Setup

```bash
pip install pillow requests python-dotenv

# Store the key without putting it in shell history:
read -rsp 'DeepInfra key: ' K && echo "DEEPINFRA_API_KEY=$K" >> ~/.env && unset K
```

The script reads `DEEPINFRA_API_KEY` from the environment, then `./.env`, then
`~/.env`.

### Options

| Flag | Purpose |
|---|---|
| `--check` | Verify the key and model without an image |
| `--crop x,y,w,h` | Crop before sending, so small details survive downscaling |
| `--max-dim N` | Override the 1280 px limit |
| `--model NAME` | Override the model |
| `--json` | Machine-readable output |

Images larger than 1280 px are downscaled to keep vision token costs down while
leaving GUI text legible.

### Why it is worth having

Every defect in `robot/docs/report.md` §7 was found by *measuring* something
rather than by looking at it. But "does this GUI look right" is genuinely a
visual question, and this tool answers it without breaking the session.

> Ask a specific question. "What do you see?" wastes tokens; "Read the pan value
> in the status bar and say whether the E-STOP button is red" does not.
