/*
 * sounds.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "sounds.h"
#include "comparator.h"
#include "common.h"
#include "eeprom.h"
#include "functions.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "targets.h"

#ifndef ERASED_FLASH_BYTE
#define ERASED_FLASH_BYTE  0xFF
#endif

// Runtime tones share the bridge with motor drive. All transitions are short
// critical sections; there are no waits and throttle cancellation owns priority.
#define BEEPER_REQUEST_TICKS ((LOOP_FREQUENCY_HZ + 9U) / 10U)
#define DSHOT_BEACON_FIRST 8U
#define DSHOT_BEACON_LAST 12U
#define DSHOT_BEACON_TICKS ((110U * LOOP_FREQUENCY_HZ + 999U) / 1000U)
// All deadlines are <=300 ms; signed 16-bit tick differences cover them at 20 kHz.
#if LOOP_FREQUENCY_HZ > 64000
#error Beeper tick/phase storage requires a wider type for this loop rate
#endif
static volatile uint16_t beeper_ticks;
static uint16_t beeper_requested_at;
static uint16_t beeper_deadline;
static uint8_t beeper_pending;
static uint8_t beeper_pending_repeats;
static uint8_t beeper_active;
static uint8_t beeper_segment;
static uint8_t beeper_repeats;

#define BEEPER_CARRIER_HZ 48000U
#define BEEPER_CARRIER_ARR (CPU_FREQUENCY_MHZ * 1000000U / BEEPER_CARRIER_HZ - 1U)
#define BEEPER_AUDIO_GAIN 2U
static const uint16_t beacon_frequencies[5] = { 659, 698, 784, 880, 523 };
static uint8_t beeper_polyphonic;
static uint16_t beeper_voice_hz[2];
static uint16_t beeper_voice_phase[2];
static uint16_t beep_duty_cycle;

uint8_t beep_volume;
volatile uint8_t beeping;

void setVolume(uint8_t volume)
{
    if (volume > 11) {
        volume = 11;
    }
    beep_volume = volume * 2; // volume variable from 0 - 11 equates to CCR value of 0-22
}

static void setCaptureCompare()
{
    SET_DUTY_CYCLE_ALL(beep_volume); // volume of the beep, (duty cycle) don't go
                                     // above 25 out of 2000
}

static void playBJNote(uint16_t freq, uint16_t bduration)
{ // hz and ms
    uint16_t timerOne_reload = TIM1_AUTORELOAD;

    SET_PRESCALER_PWM(10);
    timerOne_reload = CPU_FREQUENCY_MHZ*100000 / freq;

    SET_AUTO_RELOAD_PWM(timerOne_reload);
    SET_DUTY_CYCLE_ALL(beep_volume * timerOne_reload / TIM1_AUTORELOAD); // volume of the beep, (duty cycle) don't
                                                                         // go above 25 out of 2000
    delayMillis(bduration);
}

static uint16_t getBlueJayNoteFrequency(uint8_t bjarrayfreq)
{
    return 11000000 / (bjarrayfreq * 247 + 4000);
}

static void playBlueJayTune()
{
    uint8_t full_time_count = 0;
    uint16_t duration;
    uint16_t frequency;
    int i;

    comStep(3);
    // read_flash_bin(blueJayTuneBuffer , eeprom_address + 48 , 128);
    // first 4 bytes are reserved for rtttl duration, octave, beat, tempo
    for (i = 4; i < 128; i += 2) {
        RELOAD_WATCHDOG_COUNTER();
        signaltimeout = 0;

        if (eepromBuffer.tune[i] == 255) {
            full_time_count++;

        } else {
            if (eepromBuffer.tune[i + 1] == 0) {
                duration = full_time_count * 254 + eepromBuffer.tune[i];
                SET_DUTY_CYCLE_ALL(0);
                delayMillis(duration);
            } else {
                frequency = getBlueJayNoteFrequency(eepromBuffer.tune[i + 1]);
                duration = ((full_time_count * 254 + eepromBuffer.tune[i]) * (100000 / frequency)) / 100;
                playBJNote(frequency, duration);
            }
            full_time_count = 0;
        }
    }
    allOff(); // turn all channels low again
    SET_PRESCALER_PWM(0); // set prescaler back to 0.
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    signaltimeout = 0;
    RELOAD_WATCHDOG_COUNTER();
}

void playStartupTune()
{
    __disable_irq();
//    beeping = 1;
    if (eepromBuffer.tune[0] != ERASED_FLASH_BYTE) {
        playBlueJayTune();
    } else {
        SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
        setCaptureCompare();
        comStep(3); // activate a pwm channel
        SET_PRESCALER_PWM(55); // frequency of beep
        delayMillis(200); // duration of beep

        comStep(5);
        SET_PRESCALER_PWM(40); // next beep is higher frequency
        delayMillis(200);

        comStep(6);
        SET_PRESCALER_PWM(25); // higher again..
        delayMillis(200);

        allOff(); // turn all channels low again
        SET_PRESCALER_PWM(0); // set prescaler back to 0.
        signaltimeout = 0;
    }

    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
  //  beeping = 0;
}

#ifdef BRUSHED_MODE
void playBrushedStartupTune()
{
    __disable_irq();
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    setCaptureCompare();
    comStep(1); // activate a pwm channel
    SET_PRESCALER_PWM(40); // frequency of beep
    delayMillis(300); // duration of beep
    comStep(2); // activate a pwm channel
    SET_PRESCALER_PWM(30); // frequency of beep
    delayMillis(300); // duration of beep
    comStep(3); // activate a pwm channel
    SET_PRESCALER_PWM(25); // frequency of beep
    delayMillis(300); // duration of beep
    comStep(4);
    SET_PRESCALER_PWM(20); // higher again..
    delayMillis(300);
    allOff(); // turn all channels low again
    SET_PRESCALER_PWM(0); // set prescaler back to 0.
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();
}

#endif // BRUSHED_MODE

static void updateBeeperVoices(void)
{
    uint16_t duty[2];
    uint16_t center = (BEEPER_CARRIER_ARR + 1U) / 2U;
    uint8_t voice;
    for (voice = 0; voice < 2; voice++) {
        beeper_voice_phase[voice] += beeper_voice_hz[voice];
        if (beeper_voice_phase[voice] >= LOOP_FREQUENCY_HZ) {
            beeper_voice_phase[voice] -= LOOP_FREQUENCY_HZ;
        }
        if (beeper_voice_hz[voice] == 0) {
            duty[voice] = center;
        } else if (beeper_voice_phase[voice] < LOOP_FREQUENCY_HZ / 2U) {
            duty[voice] = center + beep_duty_cycle;
        } else {
            duty[voice] = center - beep_duty_cycle;
        }
    }
    setPWMCompare1(duty[0]);
    setPWMCompare2(center);
    setPWMCompare3(duty[1]);
}

uint32_t lockMotorOutputs(void)
{
#ifdef MCU_CH32V203
    uint32_t mask;
    __asm volatile ("csrr %0, 0x800" : "=r" (mask));
#else
    uint32_t mask = __get_PRIMASK();
#endif
    __disable_irq();
    return mask;
}

void unlockMotorOutputs(uint32_t mask)
{
#ifdef MCU_CH32V203
    __asm volatile ("csrw 0x800, %0" : : "r" (mask) : "memory");
#else
    __set_PRIMASK(mask);
#endif
}

// Shared by expiry, cancellation and note changes; keep one restore sequence.
static __attribute__((noinline)) void stopRuntimeTone(void)
{
    beeper_pending = 0;
    beeper_polyphonic = 0;
    beeper_voice_phase[0] = 0;
    beeper_voice_phase[1] = 0;
    if (beeper_active) {
        SET_DUTY_CYCLE_ALL(0);
        allOff();
        SET_PRESCALER_PWM(0);
        SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
        // PSC/ARR/CCR may be buffered. Latch motor timing and zero duty while
        // all phases are floating, before releasing ownership of the bridge.
        generatePwmTimerEvent();
        beeper_active = 0;
        beeping = 0;
    }
}

void cancelRuntimeTone(void)
{
    uint32_t mask = lockMotorOutputs();
    stopRuntimeTone();
    unlockMotorOutputs(mask);
}

static void startBeeperSegment(void);

static __attribute__((noinline)) void beginRuntimeTone(uint8_t tone, uint8_t repeats)
{
    // Called with interrupts masked, so telemetry cannot see stale RPM at entry.
    DISABLE_COM_TIMER_INT();
    maskPhaseInterrupts();
    allOff();
    beeper_active = tone;
    beeper_repeats = repeats;
    beeper_segment = 0;
    e_rpm = 0;
    beeping = 1;
    startBeeperSegment();
}

void requestDshotBeacon(uint8_t command)
{
    uint32_t mask;
    uint8_t tone;

    if (command < 1 || command > 5) {
        return;
    }
    mask = lockMotorOutputs();
    tone = DSHOT_BEACON_FIRST + command - 1;
    if (motorOperationActive()) {
        stopRuntimeTone();
    } else if (!beeping || beeper_active) {
        // Beacon packets describe the current note, not a playback queue.
        beeper_pending = 0;
        if (beeper_active != tone) {
            stopRuntimeTone();
            beginRuntimeTone(tone, 1);
        }
        // Same-note packets only extend the deadline: no PWM/phase restart.
        beeper_deadline = beeper_ticks + DSHOT_BEACON_TICKS;
    }
    unlockMotorOutputs(mask);
}

void requestRuntimeTone(uint8_t tone, uint8_t repeats)
{
    uint32_t mask = lockMotorOutputs();
    if (motorOperationActive()) {
        stopRuntimeTone();
    } else if ((tone == 1 || tone == 2 || tone == 6 || tone == 7) && repeats) {
        if (beeper_pending
            && (uint16_t)(beeper_ticks - beeper_requested_at) >= BEEPER_REQUEST_TICKS) {
            beeper_pending = 0;
        }
        if (!beeper_active && !beeping) {
            beeper_pending = 0;
            beginRuntimeTone(tone, repeats); // no deferred start while idle
        } else if (tone != beeper_active && !beeper_pending) {
            // One waiting request at most. Duplicates never extend its lifetime.
            beeper_pending = tone;
            beeper_pending_repeats = repeats;
            beeper_requested_at = beeper_ticks;
        }
    }
    unlockMotorOutputs(mask);
}

static void startBeeperSegment(void)
{
    uint16_t duration;
    uint16_t prescaler;
    uint16_t frequency;
    uint32_t divisor;
    uint8_t phase = 2;
#if !defined(PWM_ENABLE_BRIDGE) && LOOP_FREQUENCY_HZ >= 4000 && DEAD_TIME < 128
    uint32_t pwm_period = BEEPER_CARRIER_ARR + 1U;
    uint32_t number_of_notes;
    uint32_t duty_cycle;
    uint32_t volume_scale;
    uint8_t beep_strength = eepromBuffer.beep_volume;
#endif
#if !defined(PWM_ENABLE_BRIDGE) && LOOP_FREQUENCY_HZ >= 4000 && DEAD_TIME < 128
    if (beeper_active >= DSHOT_BEACON_FIRST && beeper_active <= DSHOT_BEACON_LAST
        && eepromBuffer.comp_pwm && eepromBuffer.driving_brake_strength == 10) {

        if (beep_strength > 11) {
            beep_strength = 5;
        }
        if (beeper_active == DSHOT_BEACON_FIRST + 2) {
            beeper_voice_hz[0] = 1047; // C6
            beeper_voice_hz[1] = 659;  // E5
            number_of_notes = 2;
        } else {
            beeper_voice_hz[0] = beacon_frequencies[beeper_active - DSHOT_BEACON_FIRST];
            beeper_voice_hz[1] = 0;
            number_of_notes = 1;
        }
        beeper_voice_phase[0] = 0;
        beeper_voice_phase[1] = 0;
        // Reference is the previous duty at slider 5 (legacy compare value 10).
        duty_cycle = (10U * BEEPER_AUDIO_GAIN * pwm_period)
            / (number_of_notes * ((uint32_t)TIM1_AUTORELOAD + 1U));
        // In the linear DTG range with PSC=0 / CKD=1, DEAD_TIME is in
        // timer-clock ticks. Add that margin AFTER scaling audio duty: its
        // fraction of the PWM period grows as the carrier frequency rises.
        // This is a duty-cycle allowance, not a change to hardware dead time.
        duty_cycle += DEAD_TIME;
        if (beeper_voice_hz[1]) {
            duty_cycle *= 2U; // double the chord's complete modulation duty
        }
        // Scale the entire reference duty, including its dead-time allowance.
        // Piecewise linear: 1=50%, 5=100%, 11=150%; zero remains silent.
        if (beep_strength == 0) {
            duty_cycle = 0;
        } else {
            // Scale of 24 means 100%. 12 is 50%, and 36 is 150%.
            // This keeps the slider steps exact without floating point.
            if (beep_strength <= 5) {
                volume_scale = 12 + 3 * (beep_strength - 1);
            } else {
                volume_scale = 24 + 2 * (beep_strength - 5);
            }
            duty_cycle = (duty_cycle * volume_scale + 12) / 24;
        }
        // Limit each voice to +/-12.5% about the shared 50% reference.
        if (duty_cycle > pwm_period / 8U) {
            duty_cycle = pwm_period / 8U;
        }
        beep_duty_cycle = duty_cycle;
        allOff();
        SET_PRESCALER_PWM(0);
        SET_AUTO_RELOAD_PWM(BEEPER_CARRIER_ARR);
        beeper_polyphonic = 1;
        updateBeeperVoices();
        generatePwmTimerEvent();
        if (beep_strength) {
            allpwm();
        }
        beeper_deadline = beeper_ticks + DSHOT_BEACON_TICKS;
        return;
    }
#endif
    // Other bridge types, nonlinear dead-time encodings and low-rate loops
    // retain the existing single-note path pending hardware-specific tuning.
    if (beeper_active >= DSHOT_BEACON_FIRST || beeper_active == 6) {
        if (beeper_active == 6) {
            frequency = 784;
        } else {
            frequency = beacon_frequencies[beeper_active - DSHOT_BEACON_FIRST];
        }
        divisor = frequency * ((uint32_t)TIM1_AUTORELOAD + 1U);
        prescaler = (CPU_FREQUENCY_MHZ * 1000000U + divisor / 2U) / divisor - 1U;
        duration = 100; // DShot requests subsequently extend this to 110 ms.
    } else if (beeper_active == 7) {
        phase = 3;
        duration = 100;
        if (beeper_segment == 3) {
            SET_DUTY_CYCLE_ALL(0);
            beeper_deadline = beeper_ticks + BEEPER_REQUEST_TICKS;
            return;
        }
        prescaler = 80 - 10 * beeper_segment;
    } else {
        if (beeper_active == 2) {
            if (beeper_segment == 0) {
                prescaler = 40;
            } else {
                prescaler = 80;
            }
        } else {
            if (beeper_segment == 0) {
                prescaler = 50;
            } else {
                prescaler = 30;
            }
        }
        duration = 150;
    }
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    SET_PRESCALER_PWM(prescaler);
    setCaptureCompare();
    generatePwmTimerEvent(); // latch the requested note immediately
    comStep(phase);
    beeper_deadline = beeper_ticks
        + ((uint32_t)duration * LOOP_FREQUENCY_HZ + 999U) / 1000U;
}

void serviceRuntimeTone(void)
{
    uint32_t mask = lockMotorOutputs();
    uint8_t tone;
    uint8_t segments;
    uint8_t next_tone;
    uint8_t next_repeats;

    beeper_ticks++;
    if (motorOperationActive()) {
        stopRuntimeTone();
    } else {
        if (beeper_pending
            && (uint16_t)(beeper_ticks - beeper_requested_at) >= BEEPER_REQUEST_TICKS) {
            beeper_pending = 0;
        }
        if (!beeper_active && beeper_pending && !beeping) {
            tone = beeper_pending;
            beeper_pending = 0;
            beginRuntimeTone(tone, beeper_pending_repeats);
        } else if (beeper_active && (int16_t)(uint16_t)(beeper_ticks - beeper_deadline) >= 0) {
            if (beeper_active == 7) {
                segments = 4;
            } else if (beeper_active >= 6) {
                segments = 1;
            } else {
                segments = 2;
            }
            if (++beeper_segment >= segments) {
                if (--beeper_repeats == 0) {
                    next_tone = beeper_pending;
                    next_repeats = beeper_pending_repeats;
                    stopRuntimeTone();
                    if (next_tone) {
                        beginRuntimeTone(next_tone, next_repeats);
                    }
                } else {
                    beeper_segment = 0;
                    startBeeperSegment();
                }
            } else {
                startBeeperSegment();
            }
        }
    }
    if (beeper_polyphonic && beeper_active) {
        updateBeeperVoices();
    }
    unlockMotorOutputs(mask);
}
