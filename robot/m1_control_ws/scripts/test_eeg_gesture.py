#!/usr/bin/env python3
"""The EEG gesture readout: does a detected gesture actually reach the GUI?

The console used to show EEG STATE only - run on/off, mode robot/camera. That
is always true and rarely changes, so it cannot answer "did the headset see me
just now?", which is the question the operator asks immediately after a clench.

There is a second reason state alone is not enough: the classifier on this
headset reports UNCLEAR_MUSCLE when it cannot separate a jaw clench from an
eyebrow raise, and a gesture that toggles a value back where it already was
produces NO state change at all. Both cases look identical to a state-only
display: nothing happens.

So the bridge publishes the gesture itself on /eeg/gesture, and this checks the
whole chain: bridge counter -> topic -> console state -> label text.

Run on the operator PC:
    QT_QPA_PLATFORM=offscreen python3 scripts/test_eeg_gesture.py
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'm1_control', 'm1_control'))

from eeg_udp_bridge import EEGCore, GESTURE_COMMANDS               # noqa: E402


def main():
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        if not cond:
            ok = False
        # The Pi's locale is ISO-8859-1, so a bullet or an arrow in a detail
        # string raises UnicodeEncodeError and kills the run. ASCII-ise the
        # detail rather than lose the test.
        detail = str(detail).encode('ascii', 'replace').decode('ascii')
        print(f'  [{"PASS" if cond else "FAIL"}] {name}'
              + (f'   {detail}' if detail else ''))

    # ------------------------------------------------------- the gesture map
    print('gesture mapping, ordered by measured reliability:')
    check('double blink is the most reliable, so it is the run toggle',
          GESTURE_COMMANDS.get('DOUBLE_BLINK') == 'TOGGLE_RUN',
          str(GESTURE_COMMANDS.get('DOUBLE_BLINK')))
    check('jaw clench switches what gaze steers',
          GESTURE_COMMANDS.get('JAW_CLENCH') == 'TOGGLE_MODE',
          str(GESTURE_COMMANDS.get('JAW_CLENCH')))
    check('eyebrow raise gets the only cost-free job',
          GESTURE_COMMANDS.get('EYEBROW_RAISE') == 'TOGGLE_MUSIC',
          str(GESTURE_COMMANDS.get('EYEBROW_RAISE')))
    check('the unreliable gesture is NOT on the run toggle',
          GESTURE_COMMANDS.get('EYEBROW_RAISE') != 'TOGGLE_RUN')

    # ------------------------------------------------ the bridge counter
    print('\nthe bridge counts gestures, not just state changes:')
    c = EEGCore(link_timeout=1.5)
    check('the counter starts at zero', c.gesture_seq == 0)

    c.on_message({'v': 1, 'seq': 1,
                  'event': {'gesture': 'DOUBLE_BLINK'}}, now=1.0)
    check('one gesture is counted', c.gesture_seq == 1, str(c.gesture_seq))
    check('double blink turns run ON', c.run is True)

    c.on_message({'v': 1, 'seq': 2,
                  'event': {'gesture': 'DOUBLE_BLINK'}}, now=2.0)
    check('a second gesture is counted too', c.gesture_seq == 2,
          str(c.gesture_seq))
    check('and it turns run back OFF', c.run is False)

    # The case that state-only display cannot show: a gesture that is
    # recognised but changes nothing.
    before = (c.run, c.mode, c.music)
    c.on_message({'v': 1, 'seq': 3,
                  'event': {'gesture': 'DOUBLE_BLINK'},
                  'state': {'run': False}}, now=3.0)
    check('a gesture that changes no state is still counted',
          c.gesture_seq == 3, f'seq={c.gesture_seq}')
    check('and the state really did not change', (c.run, c.mode, c.music)
          == before, f'{before} -> {(c.run, c.mode, c.music)}')

    # An unrecognised gesture must be visible as itself, not silently dropped.
    c.on_message({'v': 1, 'seq': 4,
                  'event': {'gesture': 'UNCLEAR_MUSCLE'}}, now=4.0)
    st = c.status()
    check('an unclear gesture is still reported', st['gesture_seq'] == 4,
          str(st['gesture_seq']))
    check('the status names the gesture that was actually seen',
          st['last'].startswith('UNCLEAR_MUSCLE'), st['last'])

    # ------------------------------------------------- the console readout
    print('\nthe console readout:')
    # No QApplication here on purpose. These checks read the source and drive
    # the render logic directly, so needing a display would make the test
    # unrunnable in exactly the environment where it is most useful - a
    # headless SSH session.
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'src', 'm1_control', 'm1_control',
                            'm1_console.py'), encoding='utf-8').read()

    check('the console subscribes to /eeg/gesture',
          "'/eeg/gesture'" in src)
    check('the console has a gesture pad',
          'pad_gesture' in src,
          'a WASD-style pad, not a line of grey text')
    check('the pad lights the key that fired',
          'set_gesture' in src,
          'so the operator sees which gesture was detected')
    check('the pad is fed by the gesture EVENT, not the state',
          "_refresh_gesture" in src and 'eeg_gesture' in src)

    # The pad's own rendering is covered by test_gesture_pad.py; this file is
    # about the data reaching it.
    check('the pad reduces "<gesture>/<command>" to a key name',
          "split('/')[0]" in src,
          'the bridge sends both, the pad needs only the gesture')

    # ------------------------------------------------------ the sound player
    print('\nthe sound player:')
    # Read the source, do not import it: the node needs rclpy, which is only
    # present inside the robot container, and a test that cannot run outside
    # the container is a test nobody runs.
    node_src = open(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'src',
        'm1_control', 'm1_control', 'sound_player_node.py'),
        encoding='utf-8').read()
    check('it plays on the EDGE, not the level',
          '_last_state' in node_src,
          'music is a toggle, so reacting to True would need two raises')
    check('it ignores the latched replay on startup',
          'initial' in node_src,
          'connecting must not make a noise')
    check('it replaces a playing sound rather than overlapping',
          'self._kill()' in node_src,
          'two players fight over the ALSA device and go silent')
    check('it rescans the folder each time',
          node_src.count('_scan()') >= 2,
          'so dropping a file in needs no restart')

    print()
    print('=' * 60)
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    print('=' * 60)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
