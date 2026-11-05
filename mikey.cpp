/*
 * Questo file fa parte del progetto PiLynx.
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
#pragma GCC optimize("O3")
#include "mikey.h"
#include "lynx_perf.h"
#include <Arduino.h>
#include <string.h>

// Trace delle letture registro (TOP READ MIKEY): fondamentali per il
// debugging, costano ~15-20% sul percorso di lettura nei loop stretti.
// 0 = "release". Con 0 la sezione TOP READ di traceDump() e' vuota
// (SERIAL e INTRSTw restano sempre attive: sono sulle write, rare).
#ifndef LYNX_TRACE_IO
#define LYNX_TRACE_IO 0
#endif

#ifndef MIKEY_MIX_SHIFT
#define MIKEY_MIX_SHIFT 5          // shift base, come originale
#endif
#ifndef MIKEY_MIX_GAIN_X16
#define MIKEY_MIX_GAIN_X16 16      // 16 = x1.0, 24 = x1.5, 32 = x2.0
#endif

Mikey::Mikey() {}

void Mikey::attachTo(cpu65sc02& cpu) {
    cpuRef = &cpu;
    cpu.attachMikey(this, &Mikey::staticRead, &Mikey::staticWrite);
}

// Le funzioni del percorso caldo (lettura/scrittura
// registri, tick dei timer, sintesi e mix audio, step per slice) girano
// dalla SRAM. Il mix audio in particolare viene chiamato per OGNI
// campione a 22050 Hz 

LYNX_RAMFUNC uint8_t Mikey::staticRead(void* ctx, uint16_t addr) {
    return static_cast<Mikey*>(ctx)->read(addr);
}
LYNX_RAMFUNC void Mikey::staticWrite(void* ctx, uint16_t addr, uint8_t value) {
    static_cast<Mikey*>(ctx)->write(addr, value);
}

LYNX_RAMFUNC int Mikey::linkTarget(int idx) {
    switch (idx) {
        case 0: return 2;
        case 1: return 3;
        case 2: return 4;
        case 3: return 5;
        case 5: return 7;
        default: return -1;
    }
}

LYNX_RAMFUNC uint8_t Mikey::irqLineMask() const {
    uint8_t mask = 0;
    for (int i = 0; i < MIKEY_TIMER_COUNT; i++) {
        if (i == 4) continue; // TIM4: bit 4 = linea dell'interrupt seriale
        if (timers[i].ctla & TCTLA_ENABLE_INTERRUPT) mask |= (uint8_t)(1 << i);
    }
    if (uart.serctl & (SERCTL_TXINT | SERCTL_RXINT)) mask |= serialIntBit;
    return mask;
}

// ============================================================================
// UART / COMLYNX
// ============================================================================
LYNX_RAMFUNC bool Mikey::tim4UartClockActive() const {
    const MikeyTimer& t = timers[4];
    if (!(t.ctla & TCTLA_ENABLE_COUNT)) return false;
    if ((t.ctla & TCTLA_PERIOD_MASK) == TCTLA_PERIOD_LINK) return false;
    if (!(t.ctla & TCTLA_ENABLE_RELOAD) && (t.ctlb & TCTLB_TIMER_DONE)) return false;
    return true;
}

LYNX_RAMFUNC void Mikey::uartStartTx(uint8_t value) {
    uart.txByte = value;
    uart.txBusy = true;
    uart.txCount++;
    uart.txTicksRemaining = MIKEY_UART_TICKS_PER_BYTE;
    uart.txCyclesElapsed = 0;
    if (!uart.rxReady) {
        intPending &= (uint8_t)~serialIntBit;
    }
}

LYNX_RAMFUNC void Mikey::uartCompleteByte() {
    uart.txBusy = false; // TXBE di nuovo attivo

    if (comlynxLoopback) {
        uart.rxByte  = uart.txByte;
        uart.rxReady = true;
        uart.rxCount++;
        assertSerialPending();
        uart.serialIrqCount++;
    } else if (uart.serctl & SERCTL_TXINT) {
        intPending |= serialIntBit;
        uart.serialIrqCount++;
    }
}

// N tick di TIM4 consumati in un colpo solo. Identico al
// decremento uno alla volta: un solo byte in volo alla volta, e i tick
// oltre la fine del byte erano comunque no-op (txBusy false).
LYNX_RAMFUNC void Mikey::uartTickBy(uint32_t ticks) {
    if (!uart.txBusy || ticks == 0) return;
    if (ticks >= uart.txTicksRemaining) {
        uart.txTicksRemaining = 0;
        uartCompleteByte();
    } else {
        uart.txTicksRemaining = (uint16_t)(uart.txTicksRemaining - ticks);
    }
}

// ============================================================================
// Lettura/scrittura registri
// ============================================================================
// (definizione piu' sotto, nella sezione Audio: serve qui per write())
static void recomputeTapMask(AudioChannel& a);

LYNX_RAMFUNC uint8_t Mikey::read(uint16_t addr) {

    uint8_t offset = (uint8_t)(addr - 0xFD00);

#if LYNX_TRACE_IO
    traceReadCount[offset]++;
    traceReadPC[offset] = cpuRef ? cpuRef->getpc() : 0;
#endif

    if (offset == MIKEY_INTSET || offset == MIKEY_INTRST) {
        assertSerialPending();
        return intPending;
    }

    if (offset == MIKEY_MIKEYHREV) return 0x01;

    if (offset < MIKEY_AUD_BASE) {
        int idx = offset >> 2;
        int reg = offset & 0x03;
        switch (reg) {
            case 0: return timers[idx].backup;
            case 1: return timers[idx].ctla;
            case 2: return timers[idx].count;
            default: return timers[idx].ctlb;
        }
    }

    if (offset >= MIKEY_AUD_BASE && offset < MIKEY_AUD_BASE + 0x20) {
        int ch = (offset - MIKEY_AUD_BASE) >> 3;
        int reg = (offset - MIKEY_AUD_BASE) & 0x07;
        AudioChannel& a = audio[ch];
        switch (reg) {
            case MIKEY_AUD_VOL:    return (uint8_t)a.volume;
            case MIKEY_AUD_SHFTFB: return (uint8_t)((a.waveshaper >> 13) & 0xFF);
            case MIKEY_AUD_OUTVAL: return (uint8_t)a.outval;
            case MIKEY_AUD_L8SHFT: return (uint8_t)(a.waveshaper & 0xFF);
            case MIKEY_AUD_TBACK:  return a.backup;
            case MIKEY_AUD_CTL:    return (uint8_t)(a.ctl | ((a.waveshaper & 0x1000) ? 0x80 : 0x00));
            case MIKEY_AUD_COUNT:  return a.count;
            default:               return (uint8_t)((a.misc & 0x0F) | ((a.waveshaper >> 4) & 0xF0));
        }
    }

    // NUOVO
    if (offset >= 0x40 && offset <= 0x43) return atten[offset - 0x40];
    if (offset == 0x44) return mpan;
    if (offset == 0x50) return mstereo;

    // ===== UART =====
    if (offset == MIKEY_SERCTL) {
        uint8_t v = SERCTL_RXD;
        if (!uart.txBusy) v |= SERCTL_TXBE;
        if (uart.rxReady) v |= SERCTL_RXRDY;
        return v;
    }
    if (offset == MIKEY_SERDAT) {
        uint8_t v = uart.rxByte;
        uart.rxReady = false;
        intPending &= (uint8_t)~serialIntBit;
        return v;
    }

    return regs[offset];
}

LYNX_RAMFUNC void Mikey::write(uint16_t addr, uint8_t value) {
    uint8_t offset = (uint8_t)(addr - 0xFD00);

    if (offset == MIKEY_CPUSLEEP) {
        bool suzyBusy = suzyRef && suzyRef->isBusy();
        if (value == 0 && suzyBusy && (intPending & irqLineMask()) == 0) {
            if (cpuRef) cpuRef->Halt(true);
        }
        regs[offset] = value;
        return;
    }

    if (offset < MIKEY_AUD_BASE) {
        int idx = offset >> 2;
        int reg = offset & 0x03;
        switch (reg) {
            case 0: timers[idx].backup = value; break;
            case 1:
                timers[idx].ctla = value & (uint8_t)~TCTLA_RESET_DONE;
                if (value & TCTLA_RESET_DONE) {
                    timers[idx].ctlb &= (uint8_t)~TCTLB_TIMER_DONE;
                }
                break;
            case 2: timers[idx].count = value; break;
            default: timers[idx].ctlb = value; break;
        }
        return;
    }

    if (offset >= MIKEY_AUD_BASE && offset < MIKEY_AUD_BASE + 0x20) {
        int ch = (offset - MIKEY_AUD_BASE) >> 3;
        int reg = (offset - MIKEY_AUD_BASE) & 0x07;
        AudioChannel& a = audio[ch];
        switch (reg) {
            case MIKEY_AUD_VOL:    a.volume = (int8_t)value; break;
            case MIKEY_AUD_SHFTFB: a.waveshaper = (a.waveshaper & 0x001FFFu) | ((uint32_t)value << 13); recomputeTapMask(a); break;
            case MIKEY_AUD_OUTVAL: a.outval = (int8_t)value; break;
            case MIKEY_AUD_L8SHFT: a.waveshaper = (a.waveshaper & 0x1FFF00u) | value; break;
            case MIKEY_AUD_TBACK:  a.backup = value; break;
            case MIKEY_AUD_CTL:
                a.ctl = value & 0x3F;
                a.waveshaper = (a.waveshaper & 0x1FEFFFu) | ((value & 0x80) ? 0x1000u : 0u);
                recomputeTapMask(a);   // bit 12 del waveshaper = switch bit 0
                if (value & TCTLA_RESET_DONE) {
                    a.misc &= (uint8_t)~TCTLB_TIMER_DONE;
                }
                // un write con bit reload o count-enable
                // riallinea la fase del contatore (usAccum) al momento della
                // scrittura, invece di lasciare il resto accumulato prima.
                if (value & 0x48) a.usAccum = 0;
                break;
            case MIKEY_AUD_COUNT:  a.count = value; break;
            default:
                a.misc = value;
                a.waveshaper = (a.waveshaper & 0x1FF0FFu) | (((uint32_t)value & 0xF0u) << 4);
                break;
        }
        return;
    }

    // registri stereo/pan/attenuazione (Lynx II, ma letti/scritti
    // anche su Lynx I: se il gioco li usa per silenziare/pannare un canale
    // e vengono ignorati, quel canale resta sempre nel mix per intero).
    if (offset >= 0x40 && offset <= 0x43) { atten[offset - 0x40] = value; return; }
    if (offset == 0x44) { mpan = value; return; }
    if (offset == 0x50) { mstereo = value; return; }

    if (offset == MIKEY_INTRST) {
        intPending &= (uint8_t)~value;
        intrstWrites++;
        lastIntrst = value;
        return;
    }
    if (offset == MIKEY_SYSCTL1 || offset == MIKEY_IODAT) {
        regs[offset] = value;
        if (cartBankWrite) cartBankWrite(cartCtx, offset, value);
        return;
    }
    if (offset == MIKEY_INTSET) {
        if (value != 0) intPending |= value;
        return;
    }

    // ===== UART =====
    if (offset == MIKEY_SERCTL) {
        uart.serctl = value;
        regs[offset] = value;
        return;
    }
    if (offset == MIKEY_SERDAT) {
        serdatWrites++;
        regs[MIKEY_SERDAT] = value;
        uartStartTx(value);
        return;
    }

    regs[offset] = value;
}

// ============================================================================
// Timer
// ============================================================================
LYNX_RAMFUNC void Mikey::timerUnderflow(int idx) {
    underflowCount++;
    underflowPerTimer[idx]++;

    // Pending gated dal bit 7 di CTLA. TIM4: i suoi underflow sono i tick
    // 16x della UART e il suo bit INTSET appartiene al seriale.
    if (idx == 4) {
        if (uart.txBusy && uart.txTicksRemaining > 0) {
            if (--uart.txTicksRemaining == 0) uartCompleteByte();
        }
    } else if (timers[idx].ctla & TCTLA_ENABLE_INTERRUPT) {
        intPending |= (uint8_t)(1 << idx);
    }

    if (idx == 2) vblFlag = true;

    timers[idx].ctlb |= TCTLB_TIMER_DONE;

    int target = linkTarget(idx);
    if (target >= 0) {
        MikeyTimer& t = timers[target];
        if ((t.ctla & TCTLA_ENABLE_COUNT) && (t.ctla & TCTLA_PERIOD_MASK) == TCTLA_PERIOD_LINK) {
            tickTimer(target, 1);
        }
    }

    if (idx == 7) {
        AudioChannel& a0 = audio[0];
        if ((a0.ctl & TCTLA_ENABLE_COUNT) && (a0.ctl & TCTLA_PERIOD_MASK) == TCTLA_PERIOD_LINK) {
            tickAudio(0, 1);
        }
    }
}

LYNX_RAMFUNC void Mikey::tickTimer(int idx, uint32_t steps) {
    MikeyTimer& t = timers[idx];
    if (!(t.ctla & TCTLA_ENABLE_RELOAD) && (t.ctlb & TCTLB_TIMER_DONE)) return;
    if (steps == 0) return;

    if (t.ctla & TCTLA_ENABLE_RELOAD) {
        uint32_t modulus = (uint32_t)t.backup + 1;
        uint32_t pos = modulus - 1 - t.count;
        uint32_t total = pos + steps;
        uint32_t underflows = total / modulus;
        t.count = (uint8_t)(modulus - 1 - (total % modulus));
        if (underflows > MIKEY_MAX_UNDERFLOWS_PER_CALL) underflows = MIKEY_MAX_UNDERFLOWS_PER_CALL;

        if (idx == 4) {
            // BATCH: TIM4 gira a ~500 kHz come clock UART (~50-80
            // underflow per slice): passare da timerUnderflow() N volte
            // costava 2-3 ms/frame di sola contabilita'. TIM4 non tocca
            // INTSET, non ha link, non tocca audio: N underflow == N tick
            // UART + contatori, in un colpo solo.
            if (underflows > 0) {
                underflowCount += underflows;
                underflowPerTimer[4] += underflows;
                t.ctlb |= TCTLB_TIMER_DONE;
                uartTickBy(underflows);
            }
        } else {
            for (uint32_t u = 0; u < underflows; u++) timerUnderflow(idx);
        }
    } else {
        if ((uint32_t)t.count < steps) {
            t.count = 0;
            timerUnderflow(idx);
        } else {
            t.count -= (uint8_t)steps;
        }
    }
}

// ============================================================================
// Audio
// ============================================================================
static const int SWITCH_BITS[9] = {7, 0, 1, 2, 3, 4, 5, 10, 11};

// Percorso FREDDO (solo scritture registro): ricava dai bit di switch
// del waveshaper [20:12] la maschera dei tap attivi sul LFSR a 12 bit.
// Chiamato da write() per AUD_SHFTFB (bit 13-20 -> switch 1-8) e per
// AUD_CTL bit 7 (bit 12 -> switch 0). Gli altri registri audio non
// toccano i bit di switch.
static void recomputeTapMask(AudioChannel& a) {
    uint32_t switches = (a.waveshaper >> 12) & 0x1FF;
    uint16_t m = 0;
    for (int sw = 0; sw < 9; sw++) {
        if ((switches >> sw) & 1u) m |= (uint16_t)(1u << SWITCH_BITS[sw]);
    }
    a.tapMask = m;
}

// REALTIME: il nuovo bit dell'LFSR e' la parita' (NOT) dei tap
// attivi: AND con la maschera precalcolata + folding di parita' (~10 operazioni fisse su registri,
// zero accessi in flash).
LYNX_RAMFUNC uint32_t Mikey::getLfsrNext(uint32_t current, uint16_t tapMask) {
    uint32_t switches = current >> 12;              // da preservare
    uint32_t lfsr = current & 0xFFF;
    uint32_t t = lfsr & tapMask;
    t ^= t >> 8; t ^= t >> 4; t ^= t >> 2; t ^= t >> 1;  // parita' a 12 bit (folding discendente)
    uint32_t result = (t & 1u) ^ 1u;                // NOT della parita', come l'originale
    return (switches << 12) | ((lfsr << 1) & 0xFFEu) | result;
}

LYNX_RAMFUNC void Mikey::audioUnderflow(int ch) {
    AudioChannel& a = audio[ch];
    a.misc |= TCTLB_TIMER_DONE;

    if (a.volume != 0) {
        a.waveshaper = getLfsrNext(a.waveshaper, a.tapMask);
        int newBit = (int)(a.waveshaper & 1);
        int8_t volume = a.volume;
        if (a.ctl & AUDCTL_INTEGRATE) {
            int temp = a.outval;
            temp += newBit ? volume : -volume;
            if (temp > 127) temp = 127;
            if (temp < -128) temp = -128;
            a.outval = (int8_t)temp;
        } else {
            a.outval = newBit ? volume : (int8_t)(-volume);
        }
    }

    if (ch < 3) {
        AudioChannel& next = audio[ch + 1];
        if ((next.ctl & TCTLA_ENABLE_COUNT) && (next.ctl & TCTLA_PERIOD_MASK) == TCTLA_PERIOD_LINK) {
            tickAudio(ch + 1, 1);
        }
    } else {
        MikeyTimer& t1 = timers[1];
        if ((t1.ctla & TCTLA_ENABLE_COUNT) && (t1.ctla & TCTLA_PERIOD_MASK) == TCTLA_PERIOD_LINK) {
            tickTimer(1, 1);
        }
    }
}

LYNX_RAMFUNC void Mikey::tickAudio(int ch, uint32_t steps) {
    AudioChannel& a = audio[ch];

    if (!(a.ctl & TCTLA_ENABLE_RELOAD) && (a.misc & TCTLB_TIMER_DONE)) return;
    if (steps == 0) return;

    if (a.ctl & TCTLA_ENABLE_RELOAD) {
        uint32_t modulus = (uint32_t)a.backup + 1;
        uint32_t total = (uint32_t)(modulus - 1 - a.count) + steps;
        uint32_t underflows = total / modulus;
        a.count = (uint8_t)(modulus - 1 - (total % modulus));   // SEMPRE: il contatore avanza anche senza underflow
        if (underflows == 0) return;
        if (underflows > MIKEY_MAX_UNDERFLOWS_PER_CALL) underflows = MIKEY_MAX_UNDERFLOWS_PER_CALL;
        for (uint32_t u = 0; u < underflows; u++) audioUnderflow(ch);
    } else {
        if ((uint32_t)a.count < steps) { a.count = 0; audioUnderflow(ch); }
        else a.count -= (uint8_t)steps;
    }
}

LYNX_RAMFUNC int16_t Mikey::mixAudioSample() const {
    int l = 0, r = 0;
    for (int x = 0; x < 4; x++) {
        if (audioMuteMask & (1 << x)) continue;   // diagnostica: canale escluso
        int v = audio[x].outval;

        // Canale escluso dal lato sinistro se il bit relativo in MSTEREO e' settato.
        if (!(mstereo & (0x10 << x))) {
            if (mpan & (0x10 << x)) l += (v * (atten[x] & 0xF0)) / 256;
            else l += v;
        }
        // Idem per il lato destro.
        if (!(mstereo & (0x01 << x))) {
            if (mpan & (0x01 << x)) r += (v * (atten[x] & 0x0F)) / 16;
            else r += v;
        }
    }

    int sum = (l + r) >> 1;   // down-mix a mono (L+R)/2, coerente col resto della pipeline
    sum = (sum << MIKEY_MIX_SHIFT);
    sum = (sum * MIKEY_MIX_GAIN_X16) >> 4;
    if (sum < -32768) sum = -32768;
    if (sum > 32767) sum = 32767;
    return (int16_t)sum;
}

// ============================================================================
// Step per slice
// ============================================================================
LYNX_RAMFUNC void Mikey::step(int cpuCycles) {
    if (suzyRef) suzyRef->tickBusy((uint32_t)cpuCycles);

    static uint32_t cycleAccum = 0;
    cycleAccum += (uint32_t)cpuCycles;
    uint32_t wholeUs = cycleAccum >> 2;
    cycleAccum &= 0x3;

    if (wholeUs != 0) {
        for (int i = 0; i < MIKEY_TIMER_COUNT; i++) {
            MikeyTimer& t = timers[i];
            if (!(t.ctla & TCTLA_ENABLE_COUNT)) continue;
            uint8_t period = t.ctla & TCTLA_PERIOD_MASK;
            if (period == TCTLA_PERIOD_LINK) continue;

            uint32_t accum = (uint32_t)t.usAccum + wholeUs;
            uint32_t steps = accum >> period;
            t.usAccum = (uint16_t)(accum - (steps << period));
            if (steps > 0) tickTimer(i, steps);
        }

    }

    // UART: fallback (TIM4 fermo) e watchdog assoluto
    if (uart.txBusy) {
        uart.txCyclesElapsed += (uint32_t)cpuCycles;
        if (!tim4UartClockActive()) {
            if (uart.txCyclesElapsed >= MIKEY_UART_FALLBACK_CYCLES) uartCompleteByte();
        } else if (uart.txCyclesElapsed >= MIKEY_UART_WATCHDOG_CYCLES) {
            uartCompleteByte();
        }
    }

    assertSerialPending();

    if (cpuRef) {
        uint8_t effective = intPending & irqLineMask();
        if (effective != 0) {
            cpuRef->irq();
            if (cpuRef->IsHalted()) cpuRef->Halt(false);
            irqLineCount++;
        }
    }
}

// ============================================================================
// Avanzamento canali audio disaccoppiato dallo slice CPU.
// ============================================================================
LYNX_RAMFUNC void Mikey::stepAudioOnly(uint32_t us) {
    if (us == 0) return;
    for (int i = 0; i < 4; i++) {
        AudioChannel& a = audio[i];
        if (!(a.ctl & TCTLA_ENABLE_COUNT)) continue;
        uint8_t period = a.ctl & TCTLA_PERIOD_MASK;
        if (period == TCTLA_PERIOD_LINK) continue;

        uint32_t accum = (uint32_t)a.usAccum + us;
        uint32_t steps = accum >> period;
        a.usAccum = (uint16_t)(accum - (steps << period));
        if (steps > 0) tickAudio(i, steps);
    }
}

bool Mikey::consumeVblFlag() {
    bool v = vblFlag;
    vblFlag = false;
    return v;
}

// ============================================================================
// Rendering
// ============================================================================
static inline uint16_t scale4to5(uint8_t v4) { return (uint16_t)((v4 << 1) | (v4 >> 3)); }
static inline uint16_t scale4to6(uint8_t v4) { return (uint16_t)((v4 << 2) | (v4 >> 2)); }

bool Mikey::renderStripRGB565(uint16_t* outBuf, int outW, int outH, int stripY0, int stripH) {
    if (!cpuRef) return false;

    uint16_t dispAddr = (uint16_t)regs[MIKEY_DISPADDRL] | ((uint16_t)regs[MIKEY_DISPADDRH] << 8);

    uint32_t palette32[16];
    for (int i = 0; i < 16; i++) {
        uint8_t g4 = regs[MIKEY_GREEN_BASE + i] & 0x0F;
        uint8_t br = regs[MIKEY_BLUERED_BASE + i];
        uint8_t b4 = (br >> 4) & 0x0F;
        uint8_t r4 = br & 0x0F;
        uint16_t c = (uint16_t)((scale4to5(r4) << 11) | (scale4to6(g4) << 5) | scale4to5(b4));
        palette32[i] = ((uint32_t)c << 16) | c;
    }

    memset(outBuf, 0, outW * stripH * 2);

    int offY = (outH - LYNX_SCREEN_H * 2) / 2;
    const int bytesPerRow = LYNX_SCREEN_W / 2;

    int startSrcY = (stripY0 - offY) / 2;
    int endSrcY = ((stripY0 + stripH) - offY + 1) / 2;
    if (startSrcY < 0) startSrcY = 0;
    if (endSrcY > LYNX_SCREEN_H) endSrcY = LYNX_SCREEN_H;

    for (int y = startSrcY; y < endSrcY; y++) {
        uint32_t rowAddr = (uint32_t)dispAddr + (uint32_t)y * bytesPerRow;
        int outY0 = offY + y * 2 - stripY0;
        if (outY0 < 0 || outY0 >= stripH) continue;

        uint32_t* outRow0 = (uint32_t*)(outBuf + outY0 * outW);
        uint32_t* outRow1 = (outY0 + 1 < stripH) ? (uint32_t*)(outBuf + (outY0 + 1) * outW) : outRow0;

        for (int x = 0; x < LYNX_SCREEN_W; x += 2) {
            uint8_t byteVal = cpuRef->memory[(uint16_t)(rowAddr + (x >> 1))];
            uint32_t c0 = palette32[byteVal >> 4];
            uint32_t c1 = palette32[byteVal & 0x0F];
            *outRow0++ = c0; *outRow1++ = c0;
            *outRow0++ = c1; *outRow1++ = c1;
        }
    }
    return true;
}

LYNX_RAMFUNC bool Mikey::renderFrameRGB565(uint16_t* outBuf) {
    if (!cpuRef) return false;
    uint16_t dispAddr = (uint16_t)regs[MIKEY_DISPADDRL] | ((uint16_t)regs[MIKEY_DISPADDRH] << 8);
    uint16_t palette[16];
    for (int i = 0; i < 16; i++) {
        uint8_t g4 = regs[MIKEY_GREEN_BASE + i] & 0x0F;
        uint8_t br = regs[MIKEY_BLUERED_BASE + i];
        uint8_t b4 = (br >> 4) & 0x0F;
        uint8_t r4 = br & 0x0F;
        palette[i] = (uint16_t)((scale4to5(r4) << 11) | (scale4to6(g4) << 5) | scale4to5(b4));
    }
    const int bytesPerRow = LYNX_SCREEN_W / 2;
    for (int y = 0; y < LYNX_SCREEN_H; y++) {
        uint32_t rowAddr = (uint32_t)dispAddr + (uint32_t)y * bytesPerRow;
        uint16_t* outRow = outBuf + y * LYNX_SCREEN_W;
        for (int x = 0; x < LYNX_SCREEN_W; x += 2) {
            uint8_t byteVal = cpuRef->memory[(uint16_t)(rowAddr + (x >> 1))];
            *outRow++ = palette[byteVal >> 4];
            *outRow++ = palette[byteVal & 0x0F];
        }
    }
    return true;
}

// ============================================================================
// Diagnostica
// ============================================================================
void Mikey::debugStormDump() {
    Serial.println("--- MIKEY STORM DUMP ---");
    Serial.printf("intPending=%02X enable=%02X effective=%02X\n",
                  intPending, irqLineMask(), intPending & irqLineMask());
    for (int i = 0; i < MIKEY_TIMER_COUNT; i++)
        Serial.printf("T%d: ctla=%02X ctlb=%02X bak=%02X cnt=%02X uflow=%lu\n",
            i, timers[i].ctla, timers[i].ctlb, timers[i].backup, timers[i].count,
            (unsigned long)underflowPerTimer[i]);
    for (int i = 0; i < 4; i++)
        Serial.printf("AUD%d: ctl=%02X misc=%02X bak=%02X vol=%d\n",
            i, audio[i].ctl, audio[i].misc, audio[i].backup, (int)audio[i].volume);
    Serial.printf("UART: serctl=%02X txBusy=%d rxReady=%d tx=%lu rx=%lu serialIrq=%lu (loopback=%d INTbit=%02X)\n",
        uart.serctl, (int)uart.txBusy, (int)uart.rxReady,
        (unsigned long)uart.txCount, (unsigned long)uart.rxCount,
        (unsigned long)uart.serialIrqCount,
        (int)comlynxLoopback, serialIntBit);
    if (cpuRef) {
        Serial.print("Vettori $FBF0: ");
        for (int i = 0; i < 16; i += 2)
            Serial.printf("%04X ", cpuRef->memory[0xFBF0 + i] | (cpuRef->memory[0xFBF1 + i] << 8));
        Serial.printf("\nIRQvec=%04X\n",
            cpuRef->memory[0xFFFE] | (cpuRef->memory[0xFFFF] << 8));
    }
}

// Converte cicli CPU in microsecondi con resto conservato tra
// chiamate (audioCycleAccum), cosi' l'oversampling con sotto-passi
// frazionari di ciclo non fa derivare l'intonazione per troncamento.
LYNX_RAMFUNC void Mikey::stepAudioCycles(uint32_t cycles) {
    audioCycleAccum += cycles;
    uint32_t us = audioCycleAccum >> 2;   // 4 cicli CPU = 1us (CPU a 4MHz)
    audioCycleAccum &= 0x3;
    if (us) stepAudioOnly(us);
}

void Mikey::traceDump() {
    Serial.println("--- TOP READ MIKEY (1s) ---");
#if LYNX_TRACE_IO
    for (int n = 0; n < 6; n++) {
        int best = -1;
        for (int i = 0; i < 0x100; i++)
            if (best < 0 || traceReadCount[i] > traceReadCount[best]) best = i;
        if (best < 0 || traceReadCount[best] == 0) break;
        Serial.printf("  FD%02X: %10lu volte (lastPC=%04X)\n",
                      best, (unsigned long)traceReadCount[best], traceReadPC[best]);
        traceReadCount[best] = 0;
    }
#endif
    Serial.printf("  SERIAL: serctl=%02X bit=%02X tx=%lu rx=%lu irq=%lu | INTRSTw=%lu last=%02X\n",
        uart.serctl, serialIntBit,
        (unsigned long)uart.txCount, (unsigned long)uart.rxCount,
        (unsigned long)uart.serialIrqCount,
        (unsigned long)intrstWrites, lastIntrst);
}

void Mikey::dumpAudio() {
    static const char* PERIOD_NAMES[8] = {"1us","2us","4us","8us","16us","32us","64us","LINK"};
    for (int i = 0; i < 4; i++) {
        const AudioChannel& a = audio[i];
        Serial.printf("CH%d ctl=%02X per=%-4s %s%s%s bak=%3u vol=%4d cnt=%3u shift=%06lX out=%4d %s\n",
            i, a.ctl, PERIOD_NAMES[a.ctl & 0x07],
            (a.ctl & TCTLA_ENABLE_COUNT)  ? "EN "  : "   ",
            (a.ctl & TCTLA_ENABLE_RELOAD) ? "RLD " : "    ",
            (a.ctl & AUDCTL_INTEGRATE)    ? "INT"  : "   ",
            (unsigned)a.backup, (int)a.volume, (unsigned)a.count,
            (unsigned long)a.waveshaper, (int)a.outval,
            (audioMuteMask & (1 << i)) ? "[MUTED]" : "");
    }
    Serial.printf("MSTEREO=%02X MPAN=%02X ATTEN=%02X %02X %02X %02X  muteMask=%02X\n",
        mstereo, mpan, atten[0], atten[1], atten[2], atten[3], audioMuteMask);
}