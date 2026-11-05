/*
 * Questo file fa parte del progetto Pi64.
 *
 * Copyright (C) 2025 Ivan Vettori
 *
 * Questo file è opera originale dell'autore e viene rilasciato
 * sotto licenza GNU General Public License v3.0.
 * Puoi ridistribuirlo e/o modificarlo secondo i termini della GPLv3.
 *
 * Questo programma è distribuito nella speranza che sia utile,
 * ma SENZA ALCUNA GARANZIA; senza neppure la garanzia implicita
 * di COMMERCIABILITÀ o IDONEITÀ PER UN PARTICOLARE SCOPO.
 * Vedi la licenza GPLv3 per maggiori dettagli.
 */
#ifndef AUDIO_PIPELINE_H
#define AUDIO_PIPELINE_H

#include <stdint.h>
#include "ring_buffer_spsc.h"
#include "mikey.h"

#if defined(PICO_SDK_VERSION_MAJOR) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO)
    #include "pico/stdlib.h"
#else
    static inline void tight_loop_contents() {}
#endif

#define LYNX_CPU_HZ 4000000UL
#define AUDIO_RING_CAPACITY 512

using AudioRing = RingBufferSPSC<int16_t, AUDIO_RING_CAPACITY>;

typedef int16_t (*AudioSampleFn)(void* ctx);

struct AudioConfig {
    uint32_t sampleRate = 22050;
};

// ============================================================================
// CORE0: produttore
// ============================================================================
class AudioProducer {
public:
    void begin(const AudioConfig& cfg, Mikey* mikeyPtr) {
        config = cfg;
        mikeyRef = mikeyPtr;
        cyclesPerSampleQ8 = (uint32_t)(((uint64_t)LYNX_CPU_HZ << 8) / config.sampleRate);
        cycleAccumQ8 = 0;

        // cicli CPU per sotto-passo di oversampling, in Q8 (frazionario)
        // cosi' non si perde precisione per troncamento come con i microsecondi.
        subCyclesQ8 = cyclesPerSampleQ8 / OVERSAMPLE;
        subAccumQ8 = 0;

        overrideFn = nullptr;
        overrideCtx = nullptr;
    }

    // Diagnostica: forza una sorgente sample-per-sample esterna (es. il
    // tono di test 440Hz) al posto della sintesi normale oversampled da
    // Mikey.
    void setDiagnosticOverride(AudioSampleFn fn, void* fnCtx) {
        overrideFn = fn;
        overrideCtx = fnCtx;
    }

    int step(int cpuCycles, AudioRing& ring, bool waitIfFull = true) {
        if (!mikeyRef && !overrideFn) return 0;
        cycleAccumQ8 += ((uint32_t)cpuCycles << 8);
        int produced = 0;

        while (cycleAccumQ8 >= cyclesPerSampleQ8) {
            cycleAccumQ8 -= cyclesPerSampleQ8;

            int16_t sample;
            if (overrideFn) {
                sample = overrideFn(overrideCtx);
            } else {
                int32_t acc = 0;
                for (int k = 0; k < OVERSAMPLE; k++) {
                    subAccumQ8 += subCyclesQ8;
                    uint32_t c = subAccumQ8 >> 8;
                    subAccumQ8 &= 0xFF;
                    mikeyRef->stepAudioCycles(c);
                    acc += mikeyRef->mixAudioSample();
                }
                sample = (int16_t)(acc / OVERSAMPLE);
            }

            if (waitIfFull) {
                while (ring.isFull()) {
                    tight_loop_contents();
                }
                ring.push(sample);
            } else if (ring.isFull()) {
                int16_t discard;
                ring.pop(discard);
                ring.push(sample);
            } else {
                ring.push(sample);
            }
            produced++;
        }
        return produced;
    }

private:
    #ifndef AUDIO_OVERSAMPLE
      #define AUDIO_OVERSAMPLE 1
    #endif
    static const int OVERSAMPLE = AUDIO_OVERSAMPLE;

    AudioConfig config;
    Mikey* mikeyRef = nullptr;
    AudioSampleFn overrideFn = nullptr;
    void* overrideCtx = nullptr;

    uint32_t cyclesPerSampleQ8 = 0;
    uint32_t cycleAccumQ8 = 0;
    uint32_t subCyclesQ8 = 0;
    uint32_t subAccumQ8 = 0;
};

// ============================================================================
// CORE1: consumatore (in contesto IRQ)
// ============================================================================
class AudioConsumer {
public:
    void begin(void (*outFn)(int16_t sample, void* ctx), void* fnCtx) {
        writeFn = outFn;
        ctx = fnCtx;
        lastSample = 0;
        underrunActive = false;
        fadeCounter = 0;
        underrunCount = 0;
    }

    void tick(AudioRing& ring) {
        int16_t sample;
        if (ring.pop(sample)) {
            if (underrunActive) {
                fadeCounter++;
                if (fadeCounter >= FADE_IN_SAMPLES) {
                    underrunActive = false;
                    lastSample = sample;
                } else {
                    int32_t w = fadeCounter;
                    lastSample = (int16_t)(((int32_t)lastSample * (FADE_IN_SAMPLES - w)
                                           + (int32_t)sample * w) / FADE_IN_SAMPLES);
                }
            } else {
                lastSample = sample;
            }
        } else {
            if (!underrunActive) { underrunActive = true; fadeCounter = 0; }
            lastSample = (int16_t)(lastSample - (lastSample >> DECAY_SHIFT));
            underrunCount++;
        }
        if (writeFn) writeFn(lastSample, ctx);
    }

    int16_t getLastSample() const { return lastSample; }
    uint32_t getUnderrunCount() const { return underrunCount; }
    void resetUnderrunCount() { underrunCount = 0; }

private:
    static const int DECAY_SHIFT = 6;
    static const int FADE_IN_SAMPLES = 32;

    void (*writeFn)(int16_t, void*) = nullptr;
    void* ctx = nullptr;
    volatile int16_t lastSample = 0;
    bool underrunActive = false;
    int fadeCounter = 0;
    volatile uint32_t underrunCount = 0;
};

#endif