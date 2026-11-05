/*
 * Questo file fa parte del progetto PiLynx.
 *
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 *
 * Mikey: timer, sistema di interrupt, UART/ComLynx, palette, scanout
 * video e sintesi audio del Lynx.
 */

#ifndef MIKEY_H
#define MIKEY_H

#include <stdint.h>
#include "cpu65sc02.h"
#include "LynxLoader.h"
#include "suzy.h"

#define MIKEY_TIMER_COUNT 8

#define MIKEY_TIM_BASE      0x00
#define MIKEY_AUD_BASE      0x20
#define MIKEY_AUD_VOL     0
#define MIKEY_AUD_SHFTFB  1
#define MIKEY_AUD_OUTVAL  2
#define MIKEY_AUD_L8SHFT  3
#define MIKEY_AUD_TBACK   4
#define MIKEY_AUD_CTL     5
#define MIKEY_AUD_COUNT   6
#define MIKEY_AUD_MISC    7
#define MIKEY_INTRST        0x80
#define MIKEY_INTSET        0x81
#define MIKEY_MAGRDY0       0x82
#define MIKEY_MAGRDY1       0x83
#define MIKEY_AUDIN         0x86
#define MIKEY_SYSCTL1       0x87
#define MIKEY_CPUSLEEP      0x91
#define MIKEY_IODIR         0x8A
#define MIKEY_IODAT         0x8B
#define MIKEY_SERCTL        0x8C
#define MIKEY_SERDAT        0x8D
#define MIKEY_GREEN_BASE    0xA0
#define MIKEY_BLUERED_BASE  0xB0
#define MIKEY_DISPCTL       0x92
#define MIKEY_PBKUP         0x93
#define MIKEY_DISPADDRL     0x94
#define MIKEY_DISPADDRH     0x95
#define MIKEY_MIKEYHREV     0x88
#define MIKEY_MIKEYSREV     0x89

// ===== UART / COMLYNX =====
#define SERCTL_TXINT 0x80
#define SERCTL_RXINT 0x40
#define SERCTL_TXBE  0x10   // buffer TX vuoto
#define SERCTL_RXRDY 0x08   // byte ricevuto pronto
#define SERCTL_RXD   0x80   // livello linea RX (idle alta senza cavo)

// Bit INTSET dell'interrupt seriale: DEFAULT 0x10 (bit 4, linea di TIM4).
#ifndef MIKEY_INT_SERIAL
#define MIKEY_INT_SERIAL 0x10
#endif

#define MIKEY_UART_TICKS_PER_BYTE   160
#define MIKEY_UART_FALLBACK_CYCLES  1280
#define MIKEY_UART_WATCHDOG_CYCLES  400000UL

#define TCTLA_ENABLE_INTERRUPT 0x80
#define TCTLA_RESET_DONE       0x40
#define TCTLA_MAGMODE_LEGACY   0x20
#define TCTLA_ENABLE_RELOAD    0x10
#define TCTLA_ENABLE_COUNT     0x08
#define TCTLA_PERIOD_MASK      0x07
#define TCTLA_PERIOD_LINK      0x07

#define AUDCTL_INTEGRATE 0x20

#define TCTLB_TIMER_DONE   0x08
#define TCTLB_LAST_CLOCK   0x04
#define TCTLB_BORROW_IN    0x02
#define TCTLB_BORROW_OUT   0x01

#define LYNX_SCREEN_W 160
#define LYNX_SCREEN_H 102

#define MIKEY_MAX_UNDERFLOWS_PER_CALL 4096

struct MikeyTimer {
    uint8_t backup = 0;
    uint8_t ctla = 0;
    uint8_t count = 0;
    uint8_t ctlb = 0;
    uint16_t usAccum = 0;
};

struct AudioChannel {
    int8_t volume = 0;
    int8_t outval = 0;
    uint8_t backup = 0;
    uint8_t ctl = 0;
    uint8_t count = 0;
    uint8_t misc = 0;
    uint32_t waveshaper = 0;
    uint16_t usAccum = 0;
    // maschera dei tap attivi sull'LFSR a 12 bit,
    // precalcolata SOLO quando il gioco scrive i registri che toccano i
    // bit di switch del waveshaper ([20:12]). getLfsrNext() la usa per
    // calcolare il nuovo bit come parita' (lfsr & tapMask) invece di 9
    // iterazioni con shift variabile e tabella in flash PER OGNI
    // underflow audio.
    uint16_t tapMask = 0;
};

class Mikey {
public:
    Mikey();

    void attachTo(cpu65sc02& cpu);
    void attachSuzy(Suzy* s) { suzyRef = s; }
    void setRotation(LynxRotation r) { rotation = r; }

    void step(int cpuCycles);

    // Avanza SOLO i canali audio (non i timer) di 'us' microsecondi.
    // Usato da AudioProducer per l'oversampling: chiamato piu' volte per
    // ogni sample di uscita, cosi' lo shift register vede transizioni
    // intermedie invece di un unico salto a fine slice CPU.
    void stepAudioOnly(uint32_t us);

    // Avanza i canali audio dato un numero di CICLI CPU (4 cicli = 1us),
    // conservando il resto tra chiamate consecutive (audioCycleAccum).
    // Usato da AudioProducer per l'oversampling, cosi' l'intonazione non
    // deriva per troncamento (vedi audioCycleAccum sotto).
    void stepAudioCycles(uint32_t cycles);

    // Diagnostica. Bit i=1 -> canale i escluso dal mix (per isolare
    // se un canale suona correttamente da solo). Pubblico, letto/scritto
    // solo da core0 (seriale), consumato da mixAudioSample() su core1:
    // e' un singolo byte, non serve altro che dichiararlo volatile.
    volatile uint8_t audioMuteMask = 0;

    // Dump testuale dello stato dei 4 canali audio + stereo/pan su Serial.
    void dumpAudio();

    uint8_t read(uint16_t addr);
    void write(uint16_t addr, uint8_t value);

    bool renderStripRGB565(uint16_t* outBuf, int outW, int outH, int stripY0, int stripH);
    bool renderFrameRGB565(uint16_t* outBuf);

    bool consumeVblFlag();
    int16_t mixAudioSample() const;

    void attachCart(void* ctx, void (*wr)(void*, uint16_t, uint8_t)) {
        cartCtx = ctx;
        cartBankWrite = wr;
    }

    uint32_t getUnderflowCount() const { return underflowCount; }
    void resetUnderflowCount() { underflowCount = 0; }

    uint32_t underflowPerTimer[MIKEY_TIMER_COUNT] = {0};
    uint32_t irqLineCount = 0;
    void resetUnderflowPerTimer() { for (int i = 0; i < MIKEY_TIMER_COUNT; i++) underflowPerTimer[i] = 0; }
    void debugStormDump();

    uint32_t traceReadCount[0x100] = {0};
    uint16_t traceReadPC[0x100]    = {0};
    uint32_t serdatWrites = 0;
    void traceDump();

    // ===== UART/ComLynx =====
    void setComlynxLoopback(bool enable) { comlynxLoopback = enable; }
    bool comlynxLoopbackEnabled() const { return comlynxLoopback; }
    void setSerialIntBit(uint8_t bit) { serialIntBit = bit; }
    uint8_t serialIntBitValue() const { return serialIntBit; }
    uint32_t serialTxCount() const { return uart.txCount; }
    uint32_t serialRxCount() const { return uart.rxCount; }
    uint32_t serialIrqCount() const { return uart.serialIrqCount; }
    uint32_t intrstWriteCount() const { return intrstWrites; }
    uint8_t  lastIntrstValue() const { return lastIntrst; }

private:
    cpu65sc02* cpuRef = nullptr;
    Suzy* suzyRef = nullptr;
    MikeyTimer timers[MIKEY_TIMER_COUNT];
    AudioChannel audio[4];
    uint8_t intPending = 0;
    bool vblFlag = false;

    LynxRotation rotation = LynxRotation::NONE;

    uint32_t underflowCount = 0;

    uint32_t audioCycleAccum = 0;   // resto di cicli non ancora convertiti in us (stepAudioCycles)

    // Registri stereo/pan/attenuazione (0xFD40-0xFD43, 0xFD44, 0xFD50)
    uint8_t mstereo = 0;
    uint8_t mpan = 0;
    uint8_t atten[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    uint8_t regs[0x100] = {0};

    void* cartCtx = nullptr;
    void (*cartBankWrite)(void*, uint16_t, uint8_t) = nullptr;

    static uint8_t staticRead(void* ctx, uint16_t addr);
    static void staticWrite(void* ctx, uint16_t addr, uint8_t value);

    void tickTimer(int idx, uint32_t steps);
    void timerUnderflow(int idx);

    void tickAudio(int ch, uint32_t steps);
    void audioUnderflow(int ch);

    static uint32_t getLfsrNext(uint32_t current, uint16_t tapMask);
    static int linkTarget(int idx);

    uint8_t irqLineMask() const;

    // ===== UART =====
    struct UartState {
        uint8_t  serctl = 0;
        uint8_t  txByte = 0;
        uint8_t  rxByte = 0;
        bool     txBusy = false;
        bool     rxReady = false;
        uint16_t txTicksRemaining = 0;
        uint32_t txCyclesElapsed = 0;
        uint32_t txCount = 0;
        uint32_t rxCount = 0;
        uint32_t serialIrqCount = 0;
    };
    UartState uart;
    bool comlynxLoopback = true;
    uint8_t serialIntBit = MIKEY_INT_SERIAL;

    void assertSerialPending() { if (uart.rxReady) intPending |= serialIntBit; }

    uint32_t intrstWrites = 0;
    uint8_t lastIntrst = 0;

    void uartStartTx(uint8_t value);
    void uartCompleteByte();
    void uartTickBy(uint32_t ticks);   // batch di tick TIM4
    bool tim4UartClockActive() const;
};

#endif