 # Turning on procedure
 
 1. **Turn on router** located inside the cage and **connect the foot controller** using either the **magnet connector or a micro-usb cable**
 2. Connect both USB cable (teensy built-in dangling cable) and synth; **KEEP synth OFF while booting**, wait for count-down (1min); connect **USB-C OR battery** located under the synth, use a drill bit or something similar to reach the on state
 3. Connect wireless audio adaptor or a real audio cable; OPTIONAL: connect a line-in instrument
 4. Lower the volume slider, turn on the synth slowly raise volume

# Turning off procedure

> Utils -> tap tempo (one time)
> Wait till off (quick) and turn off synth

# Syncing (link vs loop1)

If jamming with friends using link then you can sync the looper and synth (ie delay) to the bpm. If jamming alone the duration of looper 1 will be use for syncing.

> Configure as LINK client: Utils -> T1 on (tap tempo works too)
> Configure as LOOPER1: Utils -> T1 off (or T1 on but no link client - *(saving cpu cycle)*) (tap tempo only for FX)

# FX
State: FX1 (right on ADC 1 - black is also connected to FX1)

 - P8: Crush wet
 - P7: Crush gain
 - P6: Echo wet
 - P5: Echo delay time (can be sync with tap tempo)
 - P4: Reverb wet (T1 on = freeze)
 - P3: Reverb room
 - P2: NA
 - P1: NA

# GENERATOR
State: Generator
Turn slowly (loud) P8: wet chaos
P7: preset

# VOCODER

Using a custom throat microphone, you can configure the synth as a vocoder (without any feedback). You can use a 1/4" splitter to get 2 mono signals (**black is ADC1 (T3 on), red is ADC2 (T4 on)**).

 1. Connect the throat microphone (turn on the switch on the PCB)
 2. Test if mic is working: T5 on (zyn) & T3 on (adc 1) - turn T3 off after test
 3. Turn on vocoder: Mixer -> T1 on
 4. Choose a zyn preset that works well with a vocoder (the default preset is bad for vocoding)
 5. You can let T5 on if you want to ear the synth with the vocoder or turn it off for vocoder only

# EQ
TODO low,mid,high. In the meantime it is used for cutting drum sample and pitch:

 - P8: drum sample duration
 - P7: pitch

# VOLUME
State: Mixer
 - P8: zynaddsubfx
 - P7: Drum samples
 - P6: ADC 1
 - P5: ADC 2
 - P4: Sooperlooper
 - P3: if link enable = metronome volume

# FOOT controller & synth layout


# Software

## Teensy (for pots and buttosn on the synth

The dangling USB cable is connected to the teensy located inside the synth, you can use a USB repeater cable to connect it to a computer and reprogrammed the teensy using the arduino sketch (notforu_teensy_synth)

## ESP32 (for foot controller and midi sync out)

*Tested with esp-idf [v4.0-1](https://github.com/espressif/esp-idf/releases/tag/v4.0-rc)*
Acting only like a Ableton Link client (cannot send tempo or start-stop) instead use a phone (Link to Midi Bridge) or Bitwig or get a friend. Open notforu_esp32_pedal/notforu/esp32 and do the usual idf build && flash.

The network is as follow: UDP client from esp32 sending to a static IP 192.168.2.199 (linux running in khadas (inside the synth) hardware is configured as static IP). ESP32 is listening on a dynamic port (sent to pd when thew first byte is received in pd). Only use to turn on/off leds.
