# NETWORK

## Wifi (for link & foot controller)

- SSID: synthorange
- PWD: synthorange 

If jamming on battery, use a phone to create a mobile hotspot instead of connecting to the router inside the box. Use the same SSID & PWD. Android IP range is hardcorded to 192.168.43.x.

## IP

Synth static IP: 192.168.43.199
Foot controller IP: 192.168.43.DYNAMIC (starting at 100)
UDP Port: 3333
Router: 192.168.43.1 - q1w2e3
 
# ON PROCEDURE
 
1. **Turn on router** located inside the box and **connect the foot controller** using either the **magnet connector or a micro-usb cable** (battery run time is ~40h)
2. Connect both USB cable (teensy built-in dangling cable) and synth; do not connect ethernet cable **KEEP synth OFF while booting**, wait for count-down (1min); connect **USB-C OR battery** (battery run time ~12h) located under the synth, use a drill bit or something similar to reach the on state
3. Connect wireless audio adaptor or a real audio cable; OPTIONAL: connect a line-in instrument
4. Lower the volume slider, turn on the synth slowly raise volume

# OFF PROCEDURE

> Utils -> tap tempo (one time)
> Wait till off (quick) and turn off synth

# SYNCING (link vs internal clock)

If jamming with friends using link then you can sync the looper and synth (ie delay) to the bpm. If jamming alone the duration of looper 1 will be use for syncing.

> Configure as LINK client: Utils -> T1 on (tap tempo works too - but not in utils == power off)
> Configure as LOOPER1: Utils -> T1 off (or T1 on but no link client - *(saving cpu cycle)*) (tap tempo useful mainly for FX delay lines)

# VOCODER

Using a custom throat microphone, you can configure the synth as a vocoder (without any feedback). You can use a 1/4" splitter to get 2 mono signals (**black is ADC1 (T3 on), red is ADC2 (T4 on)**).

 1. Connect the throat microphone (turn on the switch on the PCB)
 2. Test if mic is working: T3 on (adc 1) - turn T3 off after test
 3. Turn on vocoder: Mixer -> T1 on
 4. Choose a zyn preset that works well with a vocoder
 5. You can let T5 (Zyn) on if you want to ear the synth with the vocoder or turn it off for vocoder only, same for T3. Best sound when T5 and T3 off.


# LOOPER

Using the looper for ADC1 or ADC2:

> Utils -> T6 (swich kind hard to move right side of the big knob
> Undo = clear all loops


# FX

State: FX1 Synth / FX2 ADC

 - P8: Crush wet
 - P7: Crush gain
 - P6: Echo wet
 - P5: Echo delay time (can be sync with tap tempo)
 - P4: Reverb wet (T1 on = freeze)
 - P3: Reverb room
 - P2: BufferFX web
 - P1: BufferFX x

State: FX2 Synth / FX2 ADC

 - P8: Granule wet
 - P7: Granule feedback
 - P6: Filter web
 - P5: Filter cutoff
 - P4: Delay reverse wet
 - P3: Delay reverse cutoff
 - P2: Ceff wet
 - P1: Disto amount

# GENERATOR

 - P8: chaos wet
 - P7: chaos preset
 - P6: lira-8 wet
 - P5: lira-8 time1
 - P4: lira-8 time2
 - P3: lira-8 feedback
 - P2: lira-8 hold1
 - P1: lira-8 hold2

 Keys 60-67 Lira-8 sensor


# UTILS

> tap tempo (one time) = power off
> T1 = link vs internal clock
> T6 = record to looper ADC1 or ADC2


# SAMP-EQ

TODO low,mid,high. In the meantime it is used for cutting drum sample and pitch:

 - P8: drum sample duration
 - P7: pitch


# ARP

Using hacked version of qmidiarp (controlled by OSC from PD).

> T5 -> run clock qmidiarp
> T4 -> mute arpegio
> T3 -> mute sequencer
> T2 -> record sequencer

> P8 -> arpegio preset
> P7 -> sequencer playhead directions
> P6 -> sequencer velocity
> P5 -> sequencer note length
> P4 -> sequencer transposer
> P3 -> sequencer step length 
> P2 -> na
> P1 -> na

# MIXER

 - P8: zynaddsubfx
 - P7: Drum samples
 - P6: ADC 1
 - P5: ADC 2
 - P4: Sooperlooper
 - P3: if link enable = metronome volume

# FOOT controller & synth layout

![Marking](https://github.com/patricksebastien/notu4/blob/main/marking.jpg?raw=true)

# Software

## Teensy (for pots and buttosn on the synth

The dangling USB cable is connected to the teensy located inside the synth, you can use a USB repeater cable to connect it to a computer and reprogrammed the teensy using the arduino sketch (notforu_teensy_synth)

## ESP32 (for foot controller and midi sync out)

*Tested with esp-idf [v4.0-1](https://github.com/espressif/esp-idf/releases/tag/v4.0-rc)*
Acting only like a Ableton Link client (cannot send tempo or start-stop) instead use a phone (Link to Midi Bridge) or Bitwig or get a friend. Open notforu_esp32_pedal/notforu/esp32 and do the usual: source /home/psc/esp-idf/export.sh & idf.py build && idf.py -p /dev/ttyUSB0 flash (check cable if not listed)

The network is as follow: UDP client from esp32 sending to a static IP 192.168.43.199 (linux running in khadas (inside the synth) hardware is configured as static IP). ESP32 is listening on a dynamic port (sent to pd when thew first byte is received in pd). Only use to turn on/off leds.
