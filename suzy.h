/*
 * Questo file fa parte del progetto PiLynx.
 *
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 *
 * Suzy: coprocessore matematico, banco registri, input (joystick/
 * switches), attraversamento SCB e rendering sprite completo.
 *
 * Timing cicli di Suzy approssimato (SUZY_CYCLES_PER_PIXEL), PROCADR non modellato.
 *
 */

#ifndef SUZY_H
#define SUZY_H

#include <stdint.h>
#include "cpu65sc02.h"
#include "LynxLoader.h"

#define SUZY_SCREEN_W 160
#define SUZY_SCREEN_H 102

// Registri "di lavoro" 0xFC00-0xFC2F (offset rispetto a 0xFC00)
#define SUZY_TMPADRL   0x00
#define SUZY_TMPADRH   0x01
#define SUZY_TILTACUML 0x02
#define SUZY_TILTACUMH 0x03
#define SUZY_HOFFL     0x04
#define SUZY_HOFFH     0x05
#define SUZY_VOFFL     0x06
#define SUZY_VOFFH     0x07
#define SUZY_VIDBASL   0x08
#define SUZY_VIDBASH   0x09
#define SUZY_COLLBASL  0x0A
#define SUZY_COLLBASH  0x0B
#define SUZY_VIDADRL   0x0C
#define SUZY_VIDADRH   0x0D
#define SUZY_COLLADRL  0x0E
#define SUZY_COLLADRH  0x0F
#define SUZY_SCBNEXTL  0x10
#define SUZY_SCBNEXTH  0x11
#define SUZY_SPRDLINEL 0x12
#define SUZY_SPRDLINEH 0x13
#define SUZY_HPOSSTRTL 0x14
#define SUZY_HPOSSTRTH 0x15
#define SUZY_VPOSSTRTL 0x16
#define SUZY_VPOSSTRTH 0x17
#define SUZY_SPRHSIZL  0x18
#define SUZY_SPRHSIZH  0x19
#define SUZY_SPRVSIZL  0x1A
#define SUZY_SPRVSIZH  0x1B
#define SUZY_STRETCHL  0x1C
#define SUZY_STRETCHH  0x1D
#define SUZY_TILTL     0x1E
#define SUZY_TILTH     0x1F
#define SUZY_SPRDOFFL  0x20
#define SUZY_SPRDOFFH  0x21
#define SUZY_SPRVPOSL  0x22
#define SUZY_SPRVPOSH  0x23
#define SUZY_COLLOFFL  0x24
#define SUZY_COLLOFFH  0x25
#define SUZY_VSIZACUML 0x26
#define SUZY_VSIZACUMH 0x27
#define SUZY_HSIZOFFL  0x28
#define SUZY_HSIZOFFH  0x29
#define SUZY_VSIZOFFL  0x2A
#define SUZY_VSIZOFFH  0x2B
#define SUZY_SCBADRL   0x2C
#define SUZY_SCBADRH   0x2D
#define SUZY_PROCADRL  0x2E
#define SUZY_PROCADRH  0x2F

// Coprocessore matematico 0xFC52-0xFC6F
#define SUZY_MATHD 0x52
#define SUZY_MATHC 0x53
#define SUZY_MATHB 0x54
#define SUZY_MATHA 0x55
#define SUZY_MATHP 0x56
#define SUZY_MATHN 0x57
#define SUZY_MATHH 0x60
#define SUZY_MATHG 0x61
#define SUZY_MATHF 0x62
#define SUZY_MATHE 0x63
#define SUZY_MATHM 0x6C
#define SUZY_MATHL 0x6D
#define SUZY_MATHK 0x6E
#define SUZY_MATHJ 0x6F

// Controllo sprite 0xFC80-0xFC92
#define SUZY_SPRCTL0   0x80
#define SUZY_SPRCTL1   0x81
#define SUZY_SPRCOLL   0x82
#define SUZY_SPRINIT   0x83
#define SUZY_SUZYHREV  0x88
#define SUZY_SUZYSREV  0x89
#define SUZY_SUZYBUSEN 0x90
#define SUZY_SPRGO     0x91
#define SUZY_SPRSYS    0x92

// Input/IO 0xFCB0-0xFCC4
#define SUZY_JOYSTICK  0xB0
#define SUZY_SWITCHES  0xB1
#define SUZY_CART0     0xB2
#define SUZY_CART1     0xB3
#define SUZY_LEDS      0xC0
#define SUZY_PARSTAT   0xC2
#define SUZY_PARDATA   0xC3
#define SUZY_HOWIE     0xC4

// SPRCTL0 ($FC80)
#define SPRCTL0_BPP_MASK   0xC0
#define SPRCTL0_HFLIP      0x20
#define SPRCTL0_VFLIP      0x10
#define SPRCTL0_TYPE_MASK  0x07
#define TYPE_BACKGROUND    0x00
#define TYPE_BACKNONCOLL   0x01
#define TYPE_BSHADOW       0x02
#define TYPE_BOUNDARY      0x03
#define TYPE_NORMAL        0x04
#define TYPE_NONCOLL       0x05
#define TYPE_XOR           0x06
#define TYPE_SHADOW        0x07

// SPRCTL1 ($FC81)
#define SPRCTL1_LITERAL     0x80
#define SPRCTL1_RELOAD_MASK 0x30
#define SPRCTL1_REUSEPAL    0x08
#define SPRCTL1_SKIP        0x04
#define SPRCTL1_STARTUP     0x02
#define SPRCTL1_STARTLEFT   0x01

// SPRCOLL ($FC82)
#define SPRCOLL_DONTCOLLIDE 0x20
#define SPRCOLL_NUMBER_MASK 0x0F

// SPRGO ($FC91)
#define SPRGO_SPRITE_GO 0x01
#define SPRGO_EVERON    0x04

// SPRSYS, bit in SCRITTURA ($FC92)
#define SPRSYS_SIGNMATH   0x80
#define SPRSYS_ACCUMULATE 0x40
#define SPRSYS_NOCOLLIDE  0x20
#define SPRSYS_VSTRETCH   0x10
#define SPRSYS_LEFTHAND   0x08
#define SPRSYS_CLRUNSAFE  0x04
#define SPRSYS_STOP       0x02
// SPRSYS, bit in LETTURA ($FC92)
#define SPRSYS_MATHWORKING  0x80
#define SPRSYS_MATHWARNING  0x40
#define SPRSYS_MATHCARRY    0x20
#define SPRSYS_VSTRETCHING  0x10
#define SPRSYS_LEFTHANDED   0x08
#define SPRSYS_UNSAFE       0x04
#define SPRSYS_SPRITETOSTOP 0x02
#define SPRSYS_SPRITEWORKING 0x01

// Joypad ($FCB0)
#define JOY_RIGHT   0x10
#define JOY_LEFT    0x20
#define JOY_DOWN    0x40
#define JOY_UP      0x80
#define JOY_OPTION1 0x08
#define JOY_OPTION2 0x04
#define JOY_INNER   0x02
#define JOY_OUTER   0x01
#define SWITCH_PAUSE 0x01

#define SUZY_CYCLES_PER_PIXEL 2

class Suzy {
public:
    Suzy();

    LynxLoader* cartLoader = nullptr;

    void setRotation(LynxRotation r) { rotation = r; }

    void attachTo(cpu65sc02& cpu);

    uint8_t read(uint16_t addr);
    void write(uint16_t addr, uint8_t value);

    void setJoystick(uint8_t bits);
    void setSwitches(uint8_t bits);

    void attachCart(LynxLoader* loader) { cartLoader = loader; }

    bool isBusy() const { return busyCyclesRemaining > 0; }
    void tickBusy(uint32_t cpuCycles);

    uint32_t traceReadCount[0x100] = {0};
    uint16_t traceReadPC[0x100]    = {0};
    void traceDump();

    // callback invocata periodicamente durante runSpriteEngine() per
    // far avanzare l'audio (e SOLO l'audio, non i timer/IRQ, per evitare
    // reentrancy nella CPU: siamo gia' dentro l'esecuzione di un'istruzione
    // 6502 quando questa callback viene chiamata) in base al tempo reale
    // trascorso, cosi' un canale che stava suonando continua a farlo
    // davvero durante un disegno sprite lungo, invece di congelarsi.
    typedef void (*AudioCatchupFn)(void* ctx, uint32_t elapsedUs);
    void setAudioCatchup(AudioCatchupFn fn, void* ctx) { audioCatchupFn = fn; audioCatchupCtx = ctx; }

private:
    cpu65sc02* cpuRef = nullptr;
    uint8_t regs[0x100] = {0};

    LynxRotation rotation = LynxRotation::NONE;

    int mMathABsign = 1;
    int mMathCDsign = 1;

    bool signedMath = false, accumulate = false, noCollide = false;
    bool vstretch = false, leftHand = false, stopOnCurrent = false;
    bool mathWarning = false;
    bool spriteWorking = false;

    uint32_t busyCyclesRemaining = 0;

    uint8_t penIndex[16];

    static uint8_t staticRead(void* ctx, uint16_t addr);
    static void staticWrite(void* ctx, uint16_t addr, uint8_t value);

    inline uint16_t rd16(int off) const { return (uint16_t)regs[off] | ((uint16_t)regs[off + 1] << 8); }
    inline void wr16(int off, uint16_t v) { regs[off] = (uint8_t)(v & 0xFF); regs[off + 1] = (uint8_t)(v >> 8); }
    inline uint32_t rd32(int off) const {
        return (uint32_t)regs[off] | ((uint32_t)regs[off + 1] << 8) |
               ((uint32_t)regs[off + 2] << 16) | ((uint32_t)regs[off + 3] << 24);
    }
    inline void wr32(int off, uint32_t v) {
        regs[off] = (uint8_t)(v & 0xFF);
        regs[off + 1] = (uint8_t)((v >> 8) & 0xFF);
        regs[off + 2] = (uint8_t)((v >> 16) & 0xFF);
        regs[off + 3] = (uint8_t)((v >> 24) & 0xFF);
    }

    void doMultiply();
    void doDivide();

    uint32_t runSpriteEngine();
    void paintOneSprite(uint16_t scbAddr, uint16_t& outNextScb, uint32_t& pixelsProcessed);

    enum LineType { LINE_ERROR, LINE_LITERAL, LINE_PACKED, LINE_ABS_LITERAL };
    static const int LINE_END = -1;

    // Stato del decoder di riga: resettato da lineInit() a OGNI riga
    // (per questo drawLineT puo' copiarlo in locali senza write-back).
    uint16_t lineTmpAddr = 0;
    uint32_t lineShiftReg = 0;
    int lineShiftRegCount = 0;
    int lineRepeatCount = 0;
    int linePixel = 0;
    LineType lineType = LINE_ERROR;
    int linePacketBitsLeft = 0;
    uint16_t lineBaseAddress = 0;
    uint16_t lineCollisionAddress = 0;

    int sprCtl0PixelBits = 1;
    bool sprCtl1Literal = false;

    uint32_t lineGetBits(int bits);      // usato da lineInit (header di riga)
    uint16_t lineInit(int voff, bool haveVoff);

    // --- loop di riga specializzato per tipo ---
    void drawLineByType(uint8_t type, int hoff, int hsign, uint16_t sprhsiz,
                        uint16_t hsizacum, uint8_t collNum, bool collideEnabled,
                        int& collMax, uint32_t& pixels, int& everOn);
    template <uint8_t T>
    void drawLineT(int hoff, int hsign, uint16_t sprhsiz, uint16_t hsizacum,
                   uint8_t collNum, bool collideEnabled,
                   int& collMax, uint32_t& pixels, int& everOn);

    AudioCatchupFn audioCatchupFn = nullptr;
    void* audioCatchupCtx = nullptr;
};

#endif