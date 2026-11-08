# Getting the webcam and microphone into the VM

Step-by-step, for VMware Workstation/Player on a Windows host.

---

## First: what the VM can see right now

Run this inside the VM before changing anything.

```bash
ls /dev/video*            # webcam
arecord -l | grep ^card   # microphone
lsusb | grep -v Virtual   # real USB devices
```

Current state on this VM:

```
/dev/video*        -> NONE
arecord -l         -> card 0: AudioPCI [Ensoniq AudioPCI]   <- virtual, not real
lsusb              -> only VMware Virtual Hubs
```

So neither device is connected. That is a host-side setting, not something the
guest can fix.

---

## Part 1 — The webcam

A webcam is a USB device, so it goes through the USB path.

### Step 1 — Make sure the VM has a USB controller

1. **Power off the VM completely** (not suspend).
2. *VM → Settings → Hardware* → select **USB Controller**.
3. Tick **Show all USB input devices** (helps some cameras appear).
4. Set **USB compatibility** to **USB 3.1** if your host ports are blue/3.x,
   otherwise **USB 2.0**. Mismatching this is the most common reason a device
   never shows up.
5. If there is no USB Controller entry at all, click **Add → USB Controller**.

### Step 2 — Connect the camera while the VM is running

1. Boot the VM.
2. Plug the webcam into the **host** first. Windows will claim it.
3. In the VMware window menu: **VM → Removable Devices → \<your camera\> →
   Connect (Disconnect from Host)**.
4. Windows will make a "device disconnected" sound. That is correct — the VM
   now owns it.

### Step 3 — Verify inside the VM

```bash
lsusb                       # the camera should appear
ls /dev/video*              # expect /dev/video0 and /dev/video1
```

A UVC camera almost always creates **two** nodes (`video0` and `video1`) because
it exposes both a capture and a metadata interface. Use `video0`.

```bash
# Confirm it actually produces frames, not just that the node exists
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --all | head -20
```

If `v4l2-ctl` is missing: `sudo apt install v4l-utils`.

### Step 4 — Quick picture test

```bash
sudo apt install ffmpeg -y      # or use the ROS camera node directly
ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 /tmp/shot.jpg
ls -l /tmp/shot.jpg
```

Then look at the image. A black frame means the device is connected but the
exposure or format is wrong; a missing file means it is not really connected.

---

## Part 2 — The microphone

There are two different mechanisms depending on what you are using, and mixing
them up wastes time.

| Microphone | Path | Where to connect |
|---|---|---|
| **Laptop built-in mic** | VMware **sound device** | VM Settings → Sound Card |
| **USB headset / USB mic** | VMware **USB passthrough** | VM → Removable Devices |

### Option A — built-in microphone (sound device path)

1. Power off the VM.
2. *VM → Settings → Hardware → **Sound Card***.
3. Tick **Connect at power on**.
4. Under **Specify host sound device**, do **not** leave it on *Auto* — pick the
   actual host device explicitly. Auto often selects a virtual or HDMI device
   that captures silence.
5. Boot the VM and verify:

```bash
arecord -l
```

You should now see **two** cards: the virtual `AudioPCI` and a real one.

### Option B — USB headset (USB passthrough path)

Same as the webcam: *VM → Removable Devices → \<headset\> → Connect*. It then
appears as a real sound card.

```bash
lsusb | grep -i audio
arecord -l | grep ^card
```

### Step — prove the microphone is not silent

**This is the step that matters.** A microphone can be present, openable, and
still deliver digital silence.

```bash
arecord -D plughw:2,0 -f S16_LE -r 16000 -c 1 -d 3 /tmp/mic.wav
python3 - <<'PY'
import wave, struct
w = wave.open('/tmp/mic.wav')
d = w.readframes(w.getnframes())
s = struct.unpack('<%dh' % (len(d) // 2), d)
peak = max(abs(x) for x in s) if s else 0
print(f'peak {peak} ({100*peak/32768:.1f}% of full scale)')
print('SILENT - wrong device or host not forwarding'
      if peak < 200 else 'AUDIO PRESENT')
PY
```

Replace `plughw:2,0` with whatever `arecord -l` reports.

| Peak | Meaning |
|---|---|
| `0`–`5` | Empty stream — the device is a placeholder, not a microphone |
| `~20`–`200` | A very quiet room, probably working |
| `>500` | Working — speak and it will be much higher |

The earlier measurement on this VM was `peak=1` out of 32768, which is an empty
stream, not a quiet room.

---

## Part 3 — Point the tools at the right device

Once `arecord -l` shows a real card, note its number and use it.

```bash
# Voice relay, operator side
ros2 run m1_control audio_bridge --role operator \
    --peer-ip 10.0.0.54 \
    --device-in plughw:2,0 \
    --device-out plughw:2,0
```

### The alternative that avoids all of this

The physical microphone is on the Windows host. The relay does not need ROS on
the operator side, so it can run where the hardware already is:

```bat
python audio_bridge.py --no-ros --role operator --peer-ip 10.0.0.54
```

`--no-ros` runs the same transport with no ROS and starts unmuted. If USB
passthrough keeps fighting you, this is the shorter path.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Device never appears in **Removable Devices** | No USB controller, or wrong USB version | Settings → USB Controller → set 2.0 or 3.1 to match the port |
| Device appears greyed out | Host driver holds it exclusively | Unplug, replug, then connect to VM *before* Windows grabs it |
| `/dev/video0` exists but no frames | Camera in use by the host | Close the host camera app, reconnect |
| `arecord` works but `peak=1` | Sound card on *Auto*, or a virtual device | Set the host device explicitly in Sound Card settings |
| Audio works in the VM but not in ROS | Wrong device index | Check `arecord -l` again — the number changes when devices are added |
| Everything vanishes after a reboot | **Connect at power on** not ticked | Tick it for both the USB controller and the sound card |

---

## Checklist

```
[ ] VM powered off, USB Controller present, version matches the port
[ ] Sound Card set to Connect at power on, host device chosen explicitly
[ ] VM booted
[ ] Webcam connected via VM -> Removable Devices
[ ] Headset connected via VM -> Removable Devices (USB) or Sound Card (built-in)
[ ] ls /dev/video*  shows video0
[ ] arecord -l      shows a real card, not just AudioPCI
[ ] arecord test    gives peak > 200, not 1
[ ] device numbers recorded for --device-in / --device-out
```
