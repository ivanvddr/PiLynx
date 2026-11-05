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
#include "suzy.h"
#include "lynx_perf.h"
#include <string.h>

// tutto il motore sprite (attraversamento SCB, decoder di riga, le 8 specializzazioni di drawLineT, matematica)
// gira dalla SRAM. Le 8 drawLineT da sole sono ~15-25 KB: un working
// set che da solo satura la cache XIP da 16 KB e sfratta il codice
// della CPU a ogni burst di rendering. Vedi lynx_perf.h (nota: con
// LYNX_RAM_LEVEL=1 Suzy resta in flash e resta il fallback).

Suzy::Suzy() {
    for (int i = 0; i < 16; i++) penIndex[i] = (uint8_t)i;
    wr16(SUZY_HSIZOFFL, 0x007F);
    wr16(SUZY_VSIZOFFL, 0x007F);
    // IMPOSTA LA REVISIONE SUZY A 1 PER SODDISFARE LA BOOT ROM
    regs[SUZY_SUZYHREV] = 0x01;
}

void Suzy::attachTo(cpu65sc02& cpu) {
    cpuRef = &cpu;
    cpu.attachSuzy(this, &Suzy::staticRead, &Suzy::staticWrite);
}

LYNX_RAMSPRITE uint8_t Suzy::staticRead(void* ctx, uint16_t addr) { return static_cast<Suzy*>(ctx)->read(addr); }
LYNX_RAMSPRITE void Suzy::staticWrite(void* ctx, uint16_t addr, uint8_t value) { static_cast<Suzy*>(ctx)->write(addr, value); }

void Suzy::setJoystick(uint8_t bits) { regs[SUZY_JOYSTICK] = bits; }
void Suzy::setSwitches(uint8_t bits) { regs[SUZY_SWITCHES] = bits; }

// ============================================================================
// Lettura/scrittura registri
// ============================================================================
LYNX_RAMSPRITE uint8_t Suzy::read(uint16_t addr) {
    uint8_t offset = (uint8_t)(addr - 0xFC00);

    if (offset == SUZY_SUZYHREV) {
        return 0x01;
    }

    if (offset == SUZY_CART0) {
        if (cartLoader) return cartLoader->readCart0Byte();
        return 0xFF;
    }
    if (offset == SUZY_CART1) {
        if (cartLoader) return cartLoader->readCart1Byte();
        return 0xFF;
    }

    if (offset == SUZY_JOYSTICK) {
        uint8_t j = regs[SUZY_JOYSTICK];
        uint8_t dpad = j & 0xF0;
        uint8_t rest = j & 0x0F;

        uint8_t rotDpad = dpad;
        if (rotation == LynxRotation::LEFT) {
            rotDpad = 0;
            if (dpad & JOY_UP)    rotDpad |= JOY_LEFT;
            if (dpad & JOY_LEFT)  rotDpad |= JOY_DOWN;
            if (dpad & JOY_DOWN)  rotDpad |= JOY_RIGHT;
            if (dpad & JOY_RIGHT) rotDpad |= JOY_UP;
        } else if (rotation == LynxRotation::RIGHT) {
            rotDpad = 0;
            if (dpad & JOY_UP)    rotDpad |= JOY_RIGHT;
            if (dpad & JOY_RIGHT) rotDpad |= JOY_DOWN;
            if (dpad & JOY_DOWN)  rotDpad |= JOY_LEFT;
            if (dpad & JOY_LEFT)  rotDpad |= JOY_UP;
        }

        if (!leftHand) {
            return rotDpad | rest;
        }
        uint8_t modified = rest;
        if (rotDpad & JOY_LEFT)  modified |= JOY_RIGHT;
        if (rotDpad & JOY_RIGHT) modified |= JOY_LEFT;
        if (rotDpad & JOY_UP)    modified |= JOY_DOWN;
        if (rotDpad & JOY_DOWN)  modified |= JOY_UP;
        return modified;
    }

    if (offset == SUZY_SPRSYS) {
        uint8_t v = 0;
        if (spriteWorking) v |= SPRSYS_SPRITEWORKING;
        if (stopOnCurrent) v |= SPRSYS_SPRITETOSTOP;
        if (leftHand) v |= SPRSYS_LEFTHANDED;
        if (vstretch) v |= SPRSYS_VSTRETCHING;
        if (mathWarning) v |= SPRSYS_MATHWARNING;
        return v;
    }

    return regs[offset];
}

LYNX_RAMSPRITE void Suzy::write(uint16_t addr, uint8_t value) {
    uint8_t offset = (uint8_t)(addr - 0xFC00);

    switch (offset) {
        case SUZY_TMPADRL: case SUZY_TILTACUML: case SUZY_HOFFL: case SUZY_VOFFL:
        case SUZY_VIDBASL: case SUZY_COLLBASL: case SUZY_VIDADRL: case SUZY_COLLADRL:
        case SUZY_SCBNEXTL: case SUZY_SPRDLINEL: case SUZY_HPOSSTRTL: case SUZY_VPOSSTRTL:
        case SUZY_SPRHSIZL: case SUZY_SPRVSIZL: case SUZY_STRETCHL: case SUZY_TILTL:
        case SUZY_SPRDOFFL: case SUZY_SPRVPOSL: case SUZY_COLLOFFL: case SUZY_VSIZACUML:
        case SUZY_HSIZOFFL: case SUZY_VSIZOFFL: case SUZY_SCBADRL: case SUZY_PROCADRL:
            regs[offset] = value;
            regs[offset + 1] = 0;
            return;
        default: break;
    }

    regs[offset] = value;

    switch (offset) {
        case SUZY_MATHD:
            regs[SUZY_MATHC] = 0;
            return;
        case SUZY_MATHC:
            if (signedMath) {
                uint16_t cd = rd16(SUZY_MATHD);
                if ((uint16_t)(cd - 1) & 0x8000) {
                    uint16_t conv = (uint16_t)(~cd + 1);
                    mMathCDsign = -1;
                    wr16(SUZY_MATHD, conv);
                } else {
                    mMathCDsign = 1;
                }
            }
            return;
        case SUZY_MATHB:
            regs[SUZY_MATHA] = 0;
            return;
        case SUZY_MATHA:
            if (signedMath) {
                uint16_t ab = rd16(SUZY_MATHB);
                if ((uint16_t)(ab - 1) & 0x8000) {
                    uint16_t conv = (uint16_t)(~ab + 1);
                    mMathABsign = -1;
                    wr16(SUZY_MATHB, conv);
                } else {
                    mMathABsign = 1;
                }
            }
            doMultiply();
            return;
        case SUZY_MATHP:
            regs[SUZY_MATHN] = 0;
            return;
        case SUZY_MATHH:
            regs[SUZY_MATHG] = 0;
            return;
        case SUZY_MATHF:
            regs[SUZY_MATHE] = 0;
            return;
        case SUZY_MATHE:
            doDivide();
            return;
        case SUZY_MATHK:
            regs[SUZY_MATHJ] = 0;
            return;
        case SUZY_MATHM:
            mathWarning = false;
            return;
        case SUZY_SPRSYS:
            stopOnCurrent = (value & SPRSYS_STOP) != 0;
            if (value & SPRSYS_CLRUNSAFE) { /* UNSAFE */ }
            leftHand = (value & SPRSYS_LEFTHAND) != 0;
            vstretch = (value & SPRSYS_VSTRETCH) != 0;
            noCollide = (value & SPRSYS_NOCOLLIDE) != 0;
            accumulate = (value & SPRSYS_ACCUMULATE) != 0;
            signedMath = (value & SPRSYS_SIGNMATH) != 0;
            return;
        case SUZY_SPRGO:
            if (value & SPRGO_SPRITE_GO) {
                uint32_t t0 = micros();
                uint32_t pixelsDrawn = runSpriteEngine();
                uint32_t dt = micros() - t0;
                busyCyclesRemaining = pixelsDrawn * SUZY_CYCLES_PER_PIXEL;
                spriteWorking = (busyCyclesRemaining > 0);

                static uint32_t sprgoCount = 0, sprgoTotalUs = 0, sprgoMaxUs = 0;
                sprgoCount++;
                sprgoTotalUs += dt;
                if (dt > sprgoMaxUs) sprgoMaxUs = dt;
                /*
                if (sprgoCount % 50 == 1) {
                    Serial.printf("SPRGO#%lu: pixels=%lu dt=%lu us (max visto=%lu us, media=%lu us)\n",
                                  (unsigned long)sprgoCount, (unsigned long)pixelsDrawn, (unsigned long)dt,
                                  (unsigned long)sprgoMaxUs, (unsigned long)(sprgoTotalUs / sprgoCount));
                }
                if (dt > 3000) {
                  Serial.printf("SPRGO LUNGO: pixels=%lu dt=%lu us (%.1f ms)\n",
                               (unsigned long)pixelsDrawn, (unsigned long)dt, dt / 1000.0f);
                }*/
            }
            return;
        default:
            return;
    }
}

// ============================================================================
// Coprocessore matematico
// ============================================================================
LYNX_RAMSPRITE void Suzy::doMultiply() {
    mathWarning = false;

    uint32_t ab = rd16(SUZY_MATHB);
    uint32_t cd = rd16(SUZY_MATHD);
    uint32_t result = ab * cd;
    wr32(SUZY_MATHH, result);

    if (signedMath) {
        int efghSign = mMathABsign + mMathCDsign;
        if (efghSign == 0) {
            uint32_t v = rd32(SUZY_MATHH);
            v = ~v + 1;
            wr32(SUZY_MATHH, v);
        }
    }

    if (accumulate) {
        uint32_t tmp = rd32(SUZY_MATHM) + rd32(SUZY_MATHH);
        wr32(SUZY_MATHM, tmp);
    }
}

LYNX_RAMSPRITE void Suzy::doDivide() {
    uint32_t divisor = rd16(SUZY_MATHP);
    uint32_t dividend = rd32(SUZY_MATHH);

    if (divisor != 0) {
        wr32(SUZY_MATHD, dividend / divisor);
        wr32(SUZY_MATHM, dividend % divisor);
    } else {
        wr32(SUZY_MATHD, 0xFFFFFFFFu);
        wr32(SUZY_MATHM, 0u);
        mathWarning = true;
    }
}

// ============================================================================
// Decoder pixel
// ============================================================================
LYNX_RAMSPRITE uint32_t Suzy::lineGetBits(int bits) {
    if (linePacketBitsLeft < bits) return 0;

    if (lineShiftRegCount < bits) {
        lineShiftReg <<= 24;
        lineShiftReg |= (uint32_t)cpuRef->memory[lineTmpAddr++] << 16;
        lineShiftReg |= (uint32_t)cpuRef->memory[lineTmpAddr++] << 8;
        lineShiftReg |= (uint32_t)cpuRef->memory[lineTmpAddr++];
        lineShiftRegCount += 24;
    }

    uint32_t retval = lineShiftReg >> (lineShiftRegCount - bits);
    retval &= (1u << bits) - 1;

    lineShiftRegCount -= bits;
    linePacketBitsLeft -= bits;
    return retval;
}

LYNX_RAMSPRITE uint16_t Suzy::lineInit(int voff, bool /*haveVoff*/) {
    lineShiftReg = 0;
    lineShiftRegCount = 0;
    lineRepeatCount = 0;
    linePixel = 0;
    lineType = LINE_ERROR;
    linePacketBitsLeft = 0xFFFF;

    lineTmpAddr = rd16(SUZY_SPRDLINEL);

    uint32_t offset = lineGetBits(8);

    if (offset == 0 || offset == 1) {
        linePacketBitsLeft = 0;
        lineType = LINE_PACKED;
        lineRepeatCount = 0;
        linePixel = LINE_END;
        return (uint16_t)offset;
    }

    linePacketBitsLeft = (int)((offset - 1) * 8);

    if (sprCtl1Literal) {
        lineType = LINE_ABS_LITERAL;
        lineRepeatCount = (int)(((offset - 1) * 8) / sprCtl0PixelBits);
    }

    if (voff >= 0 && voff < SUZY_SCREEN_H) {
        lineBaseAddress = (uint16_t)(rd16(SUZY_VIDBASL) + voff * (SUZY_SCREEN_W / 2));
        lineCollisionAddress = (uint16_t)(rd16(SUZY_COLLBASL) + voff * (SUZY_SCREEN_W / 2));
    }

    return (uint16_t)offset;
}

// ============================================================================
// LOOP DI RIGA SPECIALIZZATO PER TIPO
// ============================================================================
static inline void lynxPutNib(uint8_t* mem, uint16_t base, int h, int v) {
    uint16_t a = (uint16_t)(base + (h >> 1));
    uint8_t d = mem[a];
    mem[a] = (h & 1) ? (uint8_t)((d & 0xF0) | (v & 0x0F))
                     : (uint8_t)((d & 0x0F) | ((v & 0x0F) << 4));
}
static inline int lynxGetNib(const uint8_t* mem, uint16_t base, int h) {
    uint8_t d = mem[(uint16_t)(base + (h >> 1))];
    return (h & 1) ? (d & 0x0F) : (d >> 4);
}

// ============================================================================
// LOOP DI RIGA "FUSO": zero chiamate nel ciclo dei pixel.
// Il decoder e' una macchina a stati inline; l'operazione pixel e' codice
// dritto (le condizioni su T sono costanti di compilazione). Le tre macro
// garantiscono l'inline a qualunque livello. I parametri delle macro
// devono essere espressioni semplici senza side-effect (qui: 1, 4, pixBits,
// hoff, p, collNum...).
// ============================================================================
#define LYNX_GETBITS(RES_, NB_) do { \
    if (bitsLeft >= (NB_)) { \
        if (shcnt < (NB_)) { \
            shreg <<= 24; \
            shreg |= (uint32_t)mem[tmpAddr++] << 16; \
            shreg |= (uint32_t)mem[tmpAddr++] << 8; \
            shreg |= (uint32_t)mem[tmpAddr++]; \
            shcnt += 24; \
        } \
        (RES_) = (int)((shreg >> (shcnt - (NB_))) & ((1u << (NB_)) - 1u)); \
        shcnt -= (NB_); \
        bitsLeft -= (NB_); \
    } else { \
        (RES_) = 0; \
    } \
} while (0)

#define LYNX_PUTNIB(BASE_, H_, V_) do { \
    uint16_t a_ = (uint16_t)((BASE_) + ((H_) >> 1)); \
    uint8_t d_ = mem[a_]; \
    mem[a_] = ((H_) & 1) ? (uint8_t)((d_ & 0xF0) | ((V_) & 0x0F)) \
                         : (uint8_t)((d_ & 0x0F) | (((V_) & 0x0F) << 4)); \
} while (0)

#define LYNX_GETNIB(RES_, BASE_, H_) do { \
    uint8_t d_ = mem[(uint16_t)((BASE_) + ((H_) >> 1))]; \
    (RES_) = ((H_) & 1) ? (d_ & 0x0F) : (d_ >> 4); \
} while (0)

template <uint8_t T>
LYNX_RAMSPRITE_TPL void Suzy::drawLineT(int hoff, int hsign, uint16_t sprhsiz, uint16_t hsizacum,
                     uint8_t collNum, bool collideEnabled,
                     int& collMaxRef, uint32_t& pixelsRef, int& everOnRef)
{
    int collMax = collMaxRef;
    uint32_t pixels = pixelsRef;
    int everOn = everOnRef;

    uint8_t* const mem = cpuRef->memory;
    const uint16_t vid = lineBaseAddress;
    const uint16_t col = lineCollisionAddress;
    const int pixBits = sprCtl0PixelBits;
    const uint8_t* const pen = penIndex;
    const bool doColl = collideEnabled &&
        (T == TYPE_BACKGROUND || T == TYPE_BOUNDARY || T == TYPE_NORMAL ||
         T == TYPE_BSHADOW || T == TYPE_SHADOW || T == TYPE_XOR);

    // Stato del decoder: locali pure (niente catture, niente indirizzi
    // presi). lineInit lo resetta a ogni riga: nessun write-back finale.
    uint32_t shreg = lineShiftReg;
    int shcnt = lineShiftRegCount;
    int rep = lineRepeatCount;
    int lpix = linePixel;
    int bitsLeft = linePacketBitsLeft;
    LineType ltype = lineType;
    uint16_t tmpAddr = lineTmpAddr;

    // fast = sprite non scalato verso destra: la larghezza di ogni pixel
    // sorgente e' sempre 1 e hsizacum non cambia.
    const bool fast = (hsign == 1) && (sprhsiz == 0x0100) && (hsizacum < 0x0100);
    int lineGuard = 0;
    bool onscreen = false;

    while (lineGuard < 512) {
        // ---------------- DECODER: un pixel sorgente ----------------
        if (rep != 0) {
            rep--;
            if (ltype == LINE_LITERAL) {
                int v; LYNX_GETBITS(v, pixBits);
                lpix = pen[v];
            } else if (ltype == LINE_ABS_LITERAL) {
                int v; LYNX_GETBITS(v, pixBits);
                if (rep == 0 && v == 0) break; // terminatore literal-totale
                lpix = pen[v];
            }
            // LINE_PACKED: lpix resta quello del run
        } else {
            if (ltype == LINE_ABS_LITERAL) break;
            int sel; LYNX_GETBITS(sel, 1);
            ltype = sel ? LINE_LITERAL : LINE_PACKED;
            int cnt; LYNX_GETBITS(cnt, 4);
            if (ltype == LINE_LITERAL) {
                rep = cnt + 1;
                continue; // il primo pixel si fetcha al prossimo giro
            }
            if (cnt == 0) break; // marcatore fine riga packed
            int v; LYNX_GETBITS(v, pixBits);
            lpix = pen[v];
            rep = cnt; // questo pixel + cnt rimanenti = run completo
        }
        lineGuard++;

        // ---------------- EMIT: width pixel di output ----------------
        int width = 1;
        if (!fast) {
            hsizacum = (uint16_t)(hsizacum + sprhsiz);
            width = (hsizacum >> 8) & 0xFF;
            hsizacum &= 0x00FF;
        }
        for (int w = 0; w < width; w++) {
            if ((unsigned)hoff < SUZY_SCREEN_W) {
                const int p = lpix & 0x0F;

                if (T == TYPE_BACKGROUND) {          // scrive sempre; collisione senza lettura del max
                    LYNX_PUTNIB(vid, hoff, p);
                    if (doColl && p != 0x0E) LYNX_PUTNIB(col, hoff, collNum);
                } else if (T == TYPE_BACKNONCOLL) {
                    LYNX_PUTNIB(vid, hoff, p);
                } else if (T == TYPE_NONCOLL) {
                    if (p) LYNX_PUTNIB(vid, hoff, p);
                } else if (T == TYPE_BOUNDARY) {     // pen 0 e F trasparenti
                    if (p && p != 0x0F) LYNX_PUTNIB(vid, hoff, p);
                    if (p && doColl) {
                        int c; LYNX_GETNIB(c, col, hoff);
                        if (c > collMax) collMax = c;
                        LYNX_PUTNIB(col, hoff, collNum);
                    }
                } else if (T == TYPE_NORMAL) {       // collide ANCHE sul pen 0x0E (diverso da SHADOW!)
                    if (p) {
                        LYNX_PUTNIB(vid, hoff, p);
                        if (doColl) {
                            int c; LYNX_GETNIB(c, col, hoff);
                            if (c > collMax) collMax = c;
                            LYNX_PUTNIB(col, hoff, collNum);
                        }
                    }
                } else if (T == TYPE_BSHADOW) {      // pen 0, E, F trasparenti
                    if (p && p != 0x0E && p != 0x0F) LYNX_PUTNIB(vid, hoff, p);
                    if (p && p != 0x0E && doColl) {
                        int c; LYNX_GETNIB(c, col, hoff);
                        if (c > collMax) collMax = c;
                        LYNX_PUTNIB(col, hoff, collNum);
                    }
                } else if (T == TYPE_XOR) {          // XOR col background
                    if (p) {
                        int x; LYNX_GETNIB(x, vid, hoff);
                        LYNX_PUTNIB(vid, hoff, x ^ p);
                        if (p != 0x0E && doColl) {
                            int c; LYNX_GETNIB(c, col, hoff);
                            if (c > collMax) collMax = c;
                            LYNX_PUTNIB(col, hoff, collNum);
                        }
                    }
                } else if (T == TYPE_SHADOW) {       // pen 0 trasparente, pen E non collide
                    if (p) LYNX_PUTNIB(vid, hoff, p);
                    if (p && p != 0x0E && doColl) {
                        int c; LYNX_GETNIB(c, col, hoff);
                        if (c > collMax) collMax = c;
                        LYNX_PUTNIB(col, hoff, collNum);
                    }
                }

                everOn = 1;
                onscreen = true;
                pixels++;
            } else if (onscreen) {
                break; // usciti dallo schermo dopo esserci stati
            }
            hoff += hsign;
        }
    }

    collMaxRef = collMax;
    pixelsRef = pixels;
    everOnRef = everOn;
}

#undef LYNX_GETBITS
#undef LYNX_PUTNIB
#undef LYNX_GETNIB

LYNX_RAMSPRITE void Suzy::drawLineByType(uint8_t type, int hoff, int hsign, uint16_t sprhsiz,
                          uint16_t hsizacum, uint8_t collNum, bool collideEnabled,
                          int& collMax, uint32_t& pixels, int& everOn)
{
    switch (type) {
        case TYPE_BACKGROUND:  drawLineT<TYPE_BACKGROUND >(hoff, hsign, sprhsiz, hsizacum, collNum, collideEnabled, collMax, pixels, everOn); break;
        case TYPE_BACKNONCOLL: drawLineT<TYPE_BACKNONCOLL>(hoff, hsign, sprhsiz, hsizacum, collNum, collideEnabled, collMax, pixels, everOn); break;
        case TYPE_BSHADOW:     drawLineT<TYPE_BSHADOW    >(hoff, hsign, sprhsiz, hsizacum, collNum, collideEnabled, collMax, pixels, everOn); break;
        case TYPE_BOUNDARY:    drawLineT<TYPE_BOUNDARY   >(hoff, hsign, sprhsiz, hsizacum, collNum, collideEnabled, collMax, pixels, everOn); break;
        case TYPE_NORMAL:      drawLineT<TYPE_NORMAL     >(hoff, hsign, sprhsiz, hsizacum, collNum, collideEnabled, collMax, pixels, everOn); break;
        case TYPE_NONCOLL:     drawLineT<TYPE_NONCOLL    >(hoff, hsign, sprhsiz, hsizacum, collNum, collideEnabled, collMax, pixels, everOn); break;
        case TYPE_XOR:         drawLineT<TYPE_XOR        >(hoff, hsign, sprhsiz, hsizacum, collNum, collideEnabled, collMax, pixels, everOn); break;
        default:               drawLineT<TYPE_SHADOW     >(hoff, hsign, sprhsiz, hsizacum, collNum, collideEnabled, collMax, pixels, everOn); break;
    }
}

// ============================================================================
// Attraversamento SCB
// ============================================================================
LYNX_RAMSPRITE uint32_t Suzy::runSpriteEngine() {
    uint32_t pixelsProcessed = 0;

    uint16_t scbAddr = rd16(SUZY_SCBNEXTL);
    int guard = 0;
    uint32_t lastCatchupUs = micros();

    while ((scbAddr & 0xFF00) != 0 && guard++ < 4096) {
        uint16_t next;
        paintOneSprite(scbAddr, next, pixelsProcessed);
        scbAddr = next;
        if (stopOnCurrent) break;

        // Ogni 4 sprite, fa avanzare l'audio del tempo reale appena
        // trascorso (NON i timer: evitio di toccare cpuRef->irq() mentre
        // sono gia' dentro l'esecuzione di un'istruzione CPU). Il canale
        // continua a oscillare con i suoi registri correnti: la nota in
        // corso resta udibile per tutta la durata del disegno.
        if (audioCatchupFn && (guard & 0x03) == 0) {
            uint32_t now = micros();
            uint32_t dtUs = now - lastCatchupUs;
            if (dtUs > 0) {
                audioCatchupFn(audioCatchupCtx, dtUs);
                lastCatchupUs = now;
            }
        }
    }

    wr16(SUZY_SCBNEXTL, scbAddr);

    // Catchup finale per il resto del tempo trascorso nell'ultimo tratto.
    if (audioCatchupFn) {
        uint32_t now = micros();
        uint32_t dtUs = now - lastCatchupUs;
        if (dtUs > 0) audioCatchupFn(audioCatchupCtx, dtUs);
    }

    return pixelsProcessed;
}

LYNX_RAMSPRITE void Suzy::paintOneSprite(uint16_t scbAddr, uint16_t& outNextScb, uint32_t& pixelsProcessed) {
    auto rd = [&](uint16_t a) { return cpuRef->memory[a]; };
    auto rd16le = [&](uint16_t a) -> uint16_t { return (uint16_t)rd(a) | ((uint16_t)rd((uint16_t)(a + 1)) << 8); };

    uint16_t tmp = scbAddr;
    uint8_t sprctl0 = rd(tmp++);
    uint8_t sprctl1 = rd(tmp++);
    uint8_t sprcoll = rd(tmp++);
    uint16_t next = rd16le(tmp); tmp += 2;

    regs[SUZY_SPRCTL0] = sprctl0;
    regs[SUZY_SPRCTL1] = sprctl1;
    regs[SUZY_SPRCOLL] = sprcoll;
    wr16(SUZY_SCBADRL, scbAddr);

    outNextScb = next;

    if (sprctl1 & SPRCTL1_SKIP) return;

    uint8_t type = sprctl0 & SPRCTL0_TYPE_MASK;
    bool vflip = (sprctl0 & SPRCTL0_VFLIP) != 0;
    bool hflip = (sprctl0 & SPRCTL0_HFLIP) != 0;
    sprCtl0PixelBits = ((sprctl0 & SPRCTL0_BPP_MASK) >> 6) + 1;

    bool startLeft = (sprctl1 & SPRCTL1_STARTLEFT) != 0;
    bool startUp = (sprctl1 & SPRCTL1_STARTUP) != 0;
    sprCtl1Literal = (sprctl1 & SPRCTL1_LITERAL) != 0;
    uint8_t reloadDepth = (sprctl1 & SPRCTL1_RELOAD_MASK) >> 4;

    uint8_t sprcollNumber = sprcoll & SPRCOLL_NUMBER_MASK;
    bool sprcollDontCollide = (sprcoll & SPRCOLL_DONTCOLLIDE) != 0;

    // costante per TUTTO lo sprite (type/sprcollDontCollide/
    // noCollide non cambiano durante il disegno: la CPU e' ferma).
    const bool collideEnabled = !sprcollDontCollide && !noCollide &&
        (type == TYPE_BACKGROUND || type == TYPE_BOUNDARY || type == TYPE_NORMAL ||
         type == TYPE_BSHADOW || type == TYPE_SHADOW || type == TYPE_XOR);

    int mCollision = 0;

    uint16_t sprdline = rd16le(tmp); tmp += 2;
    int16_t hpos = (int16_t)rd16le(tmp); tmp += 2;
    int16_t vpos = (int16_t)rd16le(tmp); tmp += 2;
    wr16(SUZY_SPRDLINEL, sprdline);

    uint16_t sprhsiz = rd16(SUZY_SPRHSIZL), sprvsiz = rd16(SUZY_SPRVSIZL);
    uint16_t stretch = rd16(SUZY_STRETCHL), tilt = rd16(SUZY_TILTL);
    bool enableStretch = false, enableTilt = false;

    switch (reloadDepth) {
        case 1:
            sprhsiz = rd16le(tmp); tmp += 2;
            sprvsiz = rd16le(tmp); tmp += 2;
            break;
        case 2:
            enableStretch = true;
            sprhsiz = rd16le(tmp); tmp += 2;
            sprvsiz = rd16le(tmp); tmp += 2;
            stretch = rd16le(tmp); tmp += 2;
            break;
        case 3:
            enableStretch = true; enableTilt = true;
            sprhsiz = rd16le(tmp); tmp += 2;
            sprvsiz = rd16le(tmp); tmp += 2;
            stretch = rd16le(tmp); tmp += 2;
            tilt = rd16le(tmp); tmp += 2;
            break;
        default: break;
    }
    wr16(SUZY_SPRHSIZL, sprhsiz);
    wr16(SUZY_SPRVSIZL, sprvsiz);
    wr16(SUZY_STRETCHL, stretch);
    wr16(SUZY_TILTL, tilt);

    if (!(sprctl1 & SPRCTL1_REUSEPAL)) {
        for (int i = 0; i < 8; i++) {
            uint8_t b = rd(tmp++);
            penIndex[i * 2] = (uint8_t)((b >> 4) & 0x0F);
            penIndex[i * 2 + 1] = (uint8_t)(b & 0x0F);
        }
    }

    // --- Setup schermo/quadranti ---
    int16_t hoffReg = (int16_t)rd16(SUZY_HOFFL);
    int16_t voffReg = (int16_t)rd16(SUZY_VOFFL);
    int screen_h_start = hoffReg;
    int screen_h_end = hoffReg + SUZY_SCREEN_W;
    int screen_v_start = voffReg;
    int screen_v_end = voffReg + SUZY_SCREEN_H;
    int world_h_mid = screen_h_start + 0x8000 + (SUZY_SCREEN_W / 2);
    int world_v_mid = screen_v_start + 0x8000 + (SUZY_SCREEN_H / 2);

    bool superclip = (hpos < screen_h_start || hpos >= screen_h_end ||
                       vpos < screen_v_start || vpos >= screen_v_end);

    int quadrant;
    if (startLeft) quadrant = startUp ? 2 : 3;
    else           quadrant = startUp ? 1 : 0;

    int everOnScreen = 0;
    int vquadoff = 0, hquadoff = 0;

    for (int loopIdx = 0; loopIdx < 4; loopIdx++) {
        int sprite_h = hpos, sprite_v = vpos;
        int hsign = (quadrant == 0 || quadrant == 1) ? 1 : -1;
        int vsign = (quadrant == 0 || quadrant == 3) ? 1 : -1;
        if (vflip) vsign = -vsign;
        if (hflip) hsign = -hsign;

        bool render = true;
        if (superclip) {
            static const int vquadflip[4] = {1, 0, 3, 2};
            static const int hquadflip[4] = {3, 2, 1, 0};
            int modquad = quadrant;
            if (vflip) modquad = vquadflip[modquad];
            if (hflip) modquad = hquadflip[modquad];
            render = false;
            switch (modquad) {
                case 3:
                    if ((sprite_h >= screen_h_start || sprite_h < world_h_mid) && (sprite_v < screen_v_end || sprite_v > world_v_mid)) render = true;
                    break;
                case 2:
                    if ((sprite_h >= screen_h_start || sprite_h < world_h_mid) && (sprite_v >= screen_v_start || sprite_v < world_v_mid)) render = true;
                    break;
                case 1:
                    if ((sprite_h < screen_h_end || sprite_h > world_h_mid) && (sprite_v >= screen_v_start || sprite_v < world_v_mid)) render = true;
                    break;
                default:
                    if ((sprite_h < screen_h_end || sprite_h > world_h_mid) && (sprite_v < screen_v_end || sprite_v > world_v_mid)) render = true;
                    break;
            }
        }

        if (render) {
            int voff = vpos - screen_v_start;
            wr16(SUZY_TILTACUML, 0);
            uint16_t vsizacum = (vsign == 1) ? rd16(SUZY_VSIZOFFL) : 0;

            if (loopIdx == 0) vquadoff = vsign;
            if (vsign != vquadoff) voff += vsign;

            for (;;) {
                vsizacum = (uint16_t)(vsizacum + sprvsiz);
                int pixel_height = (vsizacum >> 8) & 0xFF;
                vsizacum &= 0x00FF;

                uint16_t sprdoff = lineInit(0, false);
                wr16(SUZY_SPRDOFFL, sprdoff);

                if (sprdoff == 1) { // fine quadrante
                    sprdline = (uint16_t)(sprdline + sprdoff);
                    wr16(SUZY_SPRDLINEL, sprdline);
                    break;
                }
                if (sprdoff == 0) { // fine sprite
                    loopIdx = 4;
                    break;
                }

                for (int vloop = 0; vloop < pixel_height; vloop++) {
                    if (vsign == 1 && voff >= SUZY_SCREEN_H) break;
                    if (vsign == -1 && voff < 0) break;

                    if (voff >= 0 && voff < SUZY_SCREEN_H) {
                        int16_t tiltacum = (int16_t)rd16(SUZY_TILTACUML);
                        hpos = (int16_t)(hpos + (tiltacum >> 8));
                        wr16(SUZY_TILTACUML, (uint16_t)(rd16(SUZY_TILTACUML) & 0x00FF));
                        int hoff = hpos - screen_h_start;

                        uint16_t hsizacum = (hsign == 1) ? rd16(SUZY_HSIZOFFL) : 0;

                        if (loopIdx == 0) hquadoff = hsign;
                        if (hsign != hquadoff) hoff += hsign;

                        // l'intero loop di disegno della riga e'
                        // specializzato per tipo dentro drawLineT<T>
                        lineInit(voff, true);
                        drawLineByType(type, hoff, hsign, sprhsiz, hsizacum,
                                       sprcollNumber, collideEnabled,
                                       mCollision, pixelsProcessed, everOnScreen);
                    }
                    voff += vsign;

                    if (enableStretch) {
                        sprhsiz = (uint16_t)(sprhsiz + stretch);
                        wr16(SUZY_SPRHSIZL, sprhsiz);
                    }
                    if (enableTilt) {
                        wr16(SUZY_TILTACUML, (uint16_t)(rd16(SUZY_TILTACUML) + tilt));
                    }
                }
                if (vstretch) {
                    sprvsiz = (uint16_t)(sprvsiz + stretch * pixel_height);
                    wr16(SUZY_SPRVSIZL, sprvsiz);
                }
                sprdline = (uint16_t)(sprdline + sprdoff);
                wr16(SUZY_SPRDLINEL, sprdline);
            }
        } else {
            for (;;) {
                uint16_t sprdoff = lineInit(0, false);
                sprdline = (uint16_t)(sprdline + sprdoff);
                wr16(SUZY_SPRDLINEL, sprdline);
                if (sprdoff == 1) break;
                if (sprdoff == 0) { loopIdx = 4; break; }
            }
        }

        quadrant = (quadrant + 1) & 0x03;
    }

    // Scrittura del risultato collisione
    if (collideEnabled) {
        uint16_t coloff = rd16(SUZY_COLLOFFL);
        uint16_t coldep = (uint16_t)(scbAddr + coloff);
        cpuRef->memory[coldep] = (uint8_t)mCollision;
    }
}

LYNX_RAMSPRITE void Suzy::tickBusy(uint32_t cpuCycles) {
    if (busyCyclesRemaining == 0) return;
    if (busyCyclesRemaining <= cpuCycles) {
        busyCyclesRemaining = 0;
        spriteWorking = false;
        if (cpuRef) cpuRef->Halt(false);
    } else {
        busyCyclesRemaining -= cpuCycles;
    }
}

void Suzy::traceDump() {
    Serial.println("--- TOP READ SUZY (1s) ---");
    for (int n = 0; n < 6; n++) {
        int best = -1;
        for (int i = 0; i < 0x100; i++)
            if (best < 0 || traceReadCount[i] > traceReadCount[best]) best = i;
        if (best < 0 || traceReadCount[best] == 0) break;
        Serial.printf("  FC%02X: %10lu volte (lastPC=%04X)\n",
                      best, (unsigned long)traceReadCount[best], traceReadPC[best]);
        traceReadCount[best] = 0;
    }
}