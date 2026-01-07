# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: Copyright 2026 Sam Blenny
#
# See NOTES.md for documentation links
#
from audiobusio import I2SOut
import audiofilters
from board import I2C, I2S_BCLK, I2S_DIN, I2S_MCLK, I2S_WS, A4, BUTTON3
import digitalio
from micropython import const
from pwmio import PWMOut
import synthio
import time
import ulab.numpy as np


from adafruit_tlv320 import TLV320DAC3100


# Morse Code Options
MORSE_HZ = const(622.25)

# DAC/Synth/Filter sample rate
SAMPLE_RATE = const(8000)

# I2S MCLK clock frequency
MCLK_HZ = const(15_000_000)


def configure_dac(i2c, sample_rate, mclk_hz):
    # Configure TLV320DAC (this requires a separate 15 MHz PWMOut to MCLK)

    # 1. Initialize DAC (this includes a soft reset and sets minimum volumes)
    dac = TLV320DAC3100(i2c)

    # 2. Configure headphone/speaker routing and volumes (order matters here)
    dac.speaker_output = False
    dac.headphone_output = True
    dac.dac_volume = -3  # Keep this below 0 to avoid DSP filter clipping
    dac.headphone_volume = 3  # CAUTION! Line level. Too loud for headphones!

    # 3. Configure the right PLL and CODEC settings for our sample rate
    dac.configure_clocks(sample_rate=sample_rate, mclk_freq=MCLK_HZ)

    # 4. Wait for power-on volume ramp-up to finish
    time.sleep(0.35)
    return dac


def run():

    # Set up I2C and I2S buses
    i2c = I2C()
    audio = I2SOut(bit_clock=I2S_BCLK, word_select=I2S_WS, data=I2S_DIN)

    # Set up 15 MHz MCLK PWM clock output for less hiss and distortion
    mclk_pwm = PWMOut(I2S_MCLK, frequency=MCLK_HZ, duty_cycle=2**15)

    # Initialize DAC for 8 kHz sample rate
    dac = configure_dac(i2c, SAMPLE_RATE, MCLK_HZ)

    # Build synthio sinewave patch with bandpass filter
    sinewave = np.array(
        np.sin(np.linspace(0, 2*np.pi, 640, endpoint=False)) * 30000,
        dtype=np.int16)
    synth = synthio.Synthesizer(sample_rate=SAMPLE_RATE, channel_count=1,
        envelope=None, waveform=sinewave)
    bandpass = synthio.Biquad(synthio.FilterMode.BAND_PASS, MORSE_HZ, Q=6.5)
    bp_filter = audiofilters.Filter(filter=bandpass, buffer_size=256,
        sample_rate=SAMPLE_RATE)
    audio.play(bp_filter)
    bp_filter.play(synth)
    time.sleep(0.5)  # let volume stabilize

    # Configure paddle input on tip (Left) of TRRS Jack plugged into GPIO port
    # === DANGER!!! ===
    # Plug the breakout board into the GPIO port with Sleeve in GND and Ring in
    # in A5. DO NOT accidentally plug Sleeve into 3V, which is next to GND!
    # =================
    tip = digitalio.DigitalInOut(A4)
    tip.switch_to_input(digitalio.Pull.UP)

    # Also take input from Button #3
    b3 = digitalio.DigitalInOut(BUTTON3)
    b3.switch_to_input(digitalio.Pull.UP)

    # Cache function references (go faster)
    sleep = time.sleep
    press = synth.press
    release = synth.release

    # Play sinewave on Fruit Jam DAC when Tip (Left, A4) of TRRS jack is
    # connected to GND by the Morse code key
    print(f"Code practice oscillator is ready.")
    note = synthio.Note(frequency=MORSE_HZ)
    prev_pressed = False
    while True:
        # Both Button #3 and the straight key input pin are active low, meaning
        # their value property is True when not pressed and False when pressed.
        # So, to get a value that's True when either button is pressed, take
        # the logical AND of the active low signals, then invert that with NOT.
        pressed = not (tip.value and b3.value)
        if pressed != prev_pressed:
            prev_pressed = pressed
            if pressed:
                press(note)
            else:
                release(note)
        # 1 ms debounce delay
        sleep(0.001)

run()
