/*
 * Implementazione del core 65SC02 per l'emulatore Lynx.
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 *
 */

#pragma GCC optimize("O3")
#include "cpu65sc02.h"
#include "lynx_perf.h"
#include <string.h>

#define setcarry()      (cpustatus |= FLAG_CARRY)
#define clearcarry()     (cpustatus &= ~FLAG_CARRY)
#define setzero()        (cpustatus |= FLAG_ZERO)
#define clearzero()      (cpustatus &= ~FLAG_ZERO)
#define setinterrupt()   (cpustatus |= FLAG_INTERRUPT)
#define clearinterrupt() (cpustatus &= ~FLAG_INTERRUPT)
#define setdecimal()     (cpustatus |= FLAG_DECIMAL)
#define cleardecimal()   (cpustatus &= ~FLAG_DECIMAL)
#define setoverflow()    (cpustatus |= FLAG_OVERFLOW)
#define clearoverflow()  (cpustatus &= ~FLAG_OVERFLOW)
#define setsign()        (cpustatus |= FLAG_SIGN)
#define clearsign()      (cpustatus &= ~FLAG_SIGN)

#define zerocalc(n)   { if ((n) & 0x00FF) clearzero(); else setzero(); }
#define signcalc(n)   { if ((n) & 0x0080) setsign(); else clearsign(); }
#define carrycalc(n)  { if ((n) & 0xFF00) setcarry(); else clearcarry(); }
#define overflowcalc(n, m, o) \
    { if (((n) ^ (uint16_t)(m)) & ((n) ^ (o)) & 0x0080) setoverflow(); else clearoverflow(); }

// Penalita' di pagina (+1 ciclo) per le istruzioni di LETTURA indicizzate.
// Stesso principio del core NMOS: si accumula e si somma una sola volta
// nel loop principale.
static int32_t extraCyclesAccum = 0;
#define PAGE_CROSS_PENALTY(base, eff) \
    { if (((base) ^ (eff)) & 0xFF00) extraCyclesAccum++; }

// ============================================================================
// Tabella cicli (base). Vedi il commento in testa al file .h: sono i
// valori standard WDC 65C02, usati per pragmatismo in assenza di una
// tabella cicli pubblica specifica del chip Mikey. Gli slot corrispondenti
// a RMBx/SMBx/BBRx/BBSx (assenti sul 65SC02) sono trattati come NOP con
// la stessa lunghezza in byte che avrebbero avuto sul 65C02 pieno, cosi'
// il program counter avanza in modo prevedibile anche se un binario
// dovesse (erroneamente) contenerli.
// ============================================================================
// RAM: letta per OGNI istruzione eseguita. Con LYNX_RAMDATA la tabella
// vive in SRAM (copiata all'avvio): accesso a ciclo singolo e zero
// pressione sulla cache XIP, invece di un .rodata in flash.
static const uint8_t ticktable[256] LYNX_RAMDATA = {
/*        0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F */
/*0*/     7, 6, 2, 1, 5, 3, 5, 5, 3, 2, 2, 1, 6, 4, 6, 5,
/*1*/     2, 5, 5, 1, 5, 4, 6, 5, 2, 4, 2, 1, 6, 4, 7, 5,
/*2*/     6, 6, 2, 1, 3, 3, 5, 5, 4, 2, 2, 1, 4, 4, 6, 5,
/*3*/     2, 5, 5, 1, 4, 4, 6, 5, 2, 4, 2, 1, 4, 4, 7, 5,
/*4*/     6, 6, 2, 1, 3, 3, 5, 5, 3, 2, 2, 1, 3, 4, 6, 5,
/*5*/     2, 5, 5, 1, 4, 4, 6, 5, 2, 4, 3, 1, 8, 4, 7, 5,
/*6*/     6, 6, 2, 1, 3, 3, 5, 5, 4, 2, 2, 1, 6, 4, 6, 5,
/*7*/     2, 5, 5, 1, 4, 4, 6, 5, 2, 4, 4, 1, 6, 4, 7, 5,
/*8*/     3, 6, 2, 1, 3, 3, 3, 5, 2, 2, 2, 1, 4, 4, 4, 5,
/*9*/     2, 6, 5, 1, 4, 4, 4, 5, 2, 5, 2, 1, 4, 5, 5, 5,
/*A*/     2, 6, 2, 1, 3, 3, 3, 5, 2, 2, 2, 1, 4, 4, 4, 5,
/*B*/     2, 5, 5, 1, 4, 4, 4, 5, 2, 4, 2, 1, 4, 4, 4, 5,
/*C*/     2, 6, 2, 1, 3, 3, 5, 5, 2, 2, 2, 1, 4, 4, 6, 5,
/*D*/     2, 5, 5, 1, 4, 4, 6, 5, 2, 4, 3, 1, 4, 4, 7, 5,
/*E*/     2, 6, 2, 1, 3, 3, 5, 5, 2, 2, 2, 1, 4, 4, 6, 5,
/*F*/     2, 5, 5, 1, 4, 4, 6, 5, 2, 4, 4, 1, 4, 4, 7, 5,
};

cpu65sc02::OpcodeHandler cpu65sc02::opcodeTable[256];
bool cpu65sc02::opcodeTableInitialized = false;

cpu65sc02::cpu65sc02() : pc(0), sp(0xFF), a(0), x(0), y(0), cpustatus(0), opcode(0) {}

void cpu65sc02::attachSuzy(void* ctx, MemReadHook rd, MemWriteHook wr) {
    suzyCtx = ctx; suzyRead = rd; suzyWrite = wr;
}
void cpu65sc02::attachMikey(void* ctx, MemReadHook rd, MemWriteHook wr) {
    mikeyCtx = ctx; mikeyRead = rd; mikeyWrite = wr;
}

void cpu65sc02::resetCPU() {
    if (!opcodeTableInitialized) {
        initOpcodeTable();
        opcodeTableInitialized = true;
    }
    // Niente memset qui: la RAM e' proprieta' del livello bus/system
    // (che ci carica dentro BIOS/cartuccia PRIMA del reset). Azzerarla
    // qui cancellerebbe quel lavoro. Se serve RAM pulita a "power on",
    // e' il chiamante a doverlo fare esplicitamente prima del primo
    // caricamento, una volta sola.
    a = x = y = 0;
    sp = 0xFF;
    cpustatus = FLAG_CONSTANT | FLAG_INTERRUPT;
    // Il vettore di reset va a 0xFFFC/0xFFFD come su ogni 6502: a questo
    // punto la ROM di boot (o il suo shadow) deve gia' essere mappata li'
    // dal livello bus/system, che si occupa anche del caricamento della
    // cartuccia via le porte seriali di Suzy.
    pc = (uint16_t)mem_read(0xFFFC) | ((uint16_t)mem_read(0xFFFD) << 8);
}

LYNX_RAMFUNC void cpu65sc02::nmi() {
    push16(pc);
    push8((cpustatus & ~FLAG_BREAK) | FLAG_CONSTANT);
    setinterrupt();
    cleardecimal(); // comportamento CMOS: D si azzera sempre su NMI/IRQ/BRK
    pc = (uint16_t)mem_read(0xFFFA) | ((uint16_t)mem_read(0xFFFB) << 8);
}

LYNX_RAMFUNC void cpu65sc02::irq() {
    if (cpustatus & FLAG_INTERRUPT) return;
    push16(pc);
    push8((cpustatus & ~FLAG_BREAK) | FLAG_CONSTANT);
    setinterrupt();
    cleardecimal();
    pc = (uint16_t)mem_read(0xFFFE) | ((uint16_t)mem_read(0xFFFF) << 8);
}

LYNX_RAMFUNC int cpu65sc02::execCPU(int tickcount) {
    if (halted) return tickcount;
    int32_t goal = tickcount;   
    int realtickcount = 0;
    while (goal > 0) {
        extraCyclesAccum = 0;
        opcode = mem_read(pc++);
        (this->*opcodeTable[opcode])();
        int cycles = ticktable[opcode] + extraCyclesAccum;
        goal -= cycles;
        realtickcount += cycles;
        if (halted) break;
    }
    return realtickcount;
}

// ============================================================================
// Load
// ============================================================================
LYNX_RAMFUNC void cpu65sc02::op_lda_imm()  { a = mem_read(pc++); zerocalc(a); signcalc(a); }
LYNX_RAMFUNC void cpu65sc02::op_lda_zp()   { uint16_t addr = mem_read(pc++); a = mem_read(addr); zerocalc(a); signcalc(a); }
LYNX_RAMFUNC void cpu65sc02::op_lda_zpx()  { uint16_t addr = (mem_read(pc++) + x) & 0xFF; a = mem_read(addr); zerocalc(a); signcalc(a); }
LYNX_RAMFUNC void cpu65sc02::op_lda_abs()  { uint16_t addr = mem_read(pc) | (mem_read(pc+1)<<8); pc+=2; a = mem_read(addr); zerocalc(a); signcalc(a); }
LYNX_RAMFUNC void cpu65sc02::op_lda_absx() { uint16_t b = mem_read(pc) | (mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+x; PAGE_CROSS_PENALTY(b,addr); a=mem_read(addr); zerocalc(a); signcalc(a); }
LYNX_RAMFUNC void cpu65sc02::op_lda_absy() { uint16_t b = mem_read(pc) | (mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+y; PAGE_CROSS_PENALTY(b,addr); a=mem_read(addr); zerocalc(a); signcalc(a); }
LYNX_RAMFUNC void cpu65sc02::op_lda_indx() { uint16_t zp=(mem_read(pc++)+x)&0xFF; uint16_t addr=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); a=mem_read(addr); zerocalc(a); signcalc(a); }
LYNX_RAMFUNC void cpu65sc02::op_lda_indy() { uint16_t zp=mem_read(pc++); uint16_t b=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); uint16_t addr=b+y; PAGE_CROSS_PENALTY(b,addr); a=mem_read(addr); zerocalc(a); signcalc(a); }
LYNX_RAMFUNC void cpu65sc02::op_lda_zpind(){ uint16_t zp=mem_read(pc++); uint16_t addr=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); a=mem_read(addr); zerocalc(a); signcalc(a); }

LYNX_RAMFUNC void cpu65sc02::op_ldx_imm()  { x = mem_read(pc++); zerocalc(x); signcalc(x); }
LYNX_RAMFUNC void cpu65sc02::op_ldx_zp()   { uint16_t addr=mem_read(pc++); x=mem_read(addr); zerocalc(x); signcalc(x); }
LYNX_RAMFUNC void cpu65sc02::op_ldx_zpy()  { uint16_t addr=(mem_read(pc++)+y)&0xFF; x=mem_read(addr); zerocalc(x); signcalc(x); }
LYNX_RAMFUNC void cpu65sc02::op_ldx_abs()  { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; x=mem_read(addr); zerocalc(x); signcalc(x); }
LYNX_RAMFUNC void cpu65sc02::op_ldx_absy() { uint16_t b=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+y; PAGE_CROSS_PENALTY(b,addr); x=mem_read(addr); zerocalc(x); signcalc(x); }

LYNX_RAMFUNC void cpu65sc02::op_ldy_imm()  { y = mem_read(pc++); zerocalc(y); signcalc(y); }
LYNX_RAMFUNC void cpu65sc02::op_ldy_zp()   { uint16_t addr=mem_read(pc++); y=mem_read(addr); zerocalc(y); signcalc(y); }
LYNX_RAMFUNC void cpu65sc02::op_ldy_zpx()  { uint16_t addr=(mem_read(pc++)+x)&0xFF; y=mem_read(addr); zerocalc(y); signcalc(y); }
LYNX_RAMFUNC void cpu65sc02::op_ldy_abs()  { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; y=mem_read(addr); zerocalc(y); signcalc(y); }
LYNX_RAMFUNC void cpu65sc02::op_ldy_absx() { uint16_t b=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+x; PAGE_CROSS_PENALTY(b,addr); y=mem_read(addr); zerocalc(y); signcalc(y); }

// ============================================================================
// Store (incluso STZ, nuovo CMOS)
// ============================================================================
LYNX_RAMFUNC void cpu65sc02::op_sta_zp()   { uint16_t addr=mem_read(pc++); mem_write(addr,a); }
LYNX_RAMFUNC void cpu65sc02::op_sta_zpx()  { uint16_t addr=(mem_read(pc++)+x)&0xFF; mem_write(addr,a); }
LYNX_RAMFUNC void cpu65sc02::op_sta_abs()  { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; mem_write(addr,a); }
LYNX_RAMFUNC void cpu65sc02::op_sta_absx() { uint16_t addr=(mem_read(pc)|(mem_read(pc+1)<<8))+x; pc+=2; mem_write(addr,a); }
LYNX_RAMFUNC void cpu65sc02::op_sta_absy() { uint16_t addr=(mem_read(pc)|(mem_read(pc+1)<<8))+y; pc+=2; mem_write(addr,a); }
LYNX_RAMFUNC void cpu65sc02::op_sta_indx() { uint16_t zp=(mem_read(pc++)+x)&0xFF; uint16_t addr=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); mem_write(addr,a); }
LYNX_RAMFUNC void cpu65sc02::op_sta_indy() { uint16_t zp=mem_read(pc++); uint16_t addr=(mem_read(zp)|(mem_read((zp+1)&0xFF)<<8))+y; mem_write(addr,a); }
LYNX_RAMFUNC void cpu65sc02::op_sta_zpind(){ uint16_t zp=mem_read(pc++); uint16_t addr=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); mem_write(addr,a); }

LYNX_RAMFUNC void cpu65sc02::op_stx_zp()   { uint16_t addr=mem_read(pc++); mem_write(addr,x); }
LYNX_RAMFUNC void cpu65sc02::op_stx_zpy()  { uint16_t addr=(mem_read(pc++)+y)&0xFF; mem_write(addr,x); }
LYNX_RAMFUNC void cpu65sc02::op_stx_abs()  { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; mem_write(addr,x); }

LYNX_RAMFUNC void cpu65sc02::op_sty_zp()   { uint16_t addr=mem_read(pc++); mem_write(addr,y); }
LYNX_RAMFUNC void cpu65sc02::op_sty_zpx()  { uint16_t addr=(mem_read(pc++)+x)&0xFF; mem_write(addr,y); }
LYNX_RAMFUNC void cpu65sc02::op_sty_abs()  { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; mem_write(addr,y); }

LYNX_RAMFUNC void cpu65sc02::op_stz_zp()   { uint16_t addr=mem_read(pc++); mem_write(addr,0); }
LYNX_RAMFUNC void cpu65sc02::op_stz_zpx()  { uint16_t addr=(mem_read(pc++)+x)&0xFF; mem_write(addr,0); }
LYNX_RAMFUNC void cpu65sc02::op_stz_abs()  { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; mem_write(addr,0); }
LYNX_RAMFUNC void cpu65sc02::op_stz_absx() { uint16_t addr=(mem_read(pc)|(mem_read(pc+1)<<8))+x; pc+=2; mem_write(addr,0); }

// ============================================================================
// ADC/SBC - niente bug NMOS: N/V riflettono il risultato decimale reale
// e in modalita' decimale l'istruzione costa un ciclo extra.
// ============================================================================
static inline void adc_core(uint8_t& a, uint8_t& cpustatus, uint8_t value) {
    if (cpustatus & FLAG_DECIMAL) {
        extraCyclesAccum++; // ciclo extra CMOS in modalita' decimale
        uint16_t tmp = (a & 0x0F) + (value & 0x0F) + (cpustatus & FLAG_CARRY);
        if (tmp > 9) tmp += 6;
        uint16_t hi = (a & 0xF0) + (value & 0xF0) + (tmp > 0x0F ? 0x10 : 0);
        tmp = (tmp & 0x0F) | hi;
        if ((hi) > 0x90) { tmp += 0x60; setcarry(); } else { clearcarry(); }
        zerocalc((uint8_t)((a + value + (cpustatus & FLAG_CARRY)) )); // Z su 65C02 e' dal risultato decimale
        signcalc(tmp);
        overflowcalc(tmp, a, value);
        a = (uint8_t)(tmp & 0xFF);
    } else {
        uint16_t result = (uint16_t)a + value + (uint16_t)(cpustatus & FLAG_CARRY);
        carrycalc(result);
        zerocalc(result);
        overflowcalc(result, a, value);
        signcalc(result);
        a = (uint8_t)(result & 0xFF);
    }
}

LYNX_RAMFUNC void cpu65sc02::op_adc_imm()   { adc_core(a, cpustatus, mem_read(pc++)); }
LYNX_RAMFUNC void cpu65sc02::op_adc_zp()    { uint16_t addr=mem_read(pc++); adc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_adc_zpx()   { uint16_t addr=(mem_read(pc++)+x)&0xFF; adc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_adc_abs()   { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; adc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_adc_absx()  { uint16_t b=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+x; PAGE_CROSS_PENALTY(b,addr); adc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_adc_absy()  { uint16_t b=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+y; PAGE_CROSS_PENALTY(b,addr); adc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_adc_indx()  { uint16_t zp=(mem_read(pc++)+x)&0xFF; uint16_t addr=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); adc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_adc_indy()  { uint16_t zp=mem_read(pc++); uint16_t b=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); uint16_t addr=b+y; PAGE_CROSS_PENALTY(b,addr); adc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_adc_zpind() { uint16_t zp=mem_read(pc++); uint16_t addr=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); adc_core(a, cpustatus, mem_read(addr)); }

static inline void sbc_core(uint8_t& a, uint8_t& cpustatus, uint8_t value) {
    if (cpustatus & FLAG_DECIMAL) {
        extraCyclesAccum++;
        uint16_t tmp = (a & 0x0F) - (value & 0x0F) - ((cpustatus & FLAG_CARRY) ? 0 : 1);
        if ((int16_t)tmp < 0) tmp = ((tmp - 6) & 0x0F) | ((a & 0xF0) - (value & 0xF0) - 0x10);
        else tmp = (tmp & 0x0F) | ((a & 0xF0) - (value & 0xF0));
        if ((int16_t)tmp < 0) tmp -= 0x60;
        uint16_t binResult = (uint16_t)a - value - ((cpustatus & FLAG_CARRY) ? 0 : 1);
        if (binResult < 0x100) setcarry(); else clearcarry();
        zerocalc(binResult); // Z dal risultato binario, come su NMOS/CMOS
        signcalc(tmp);
        overflowcalc(tmp, a, value);
        a = (uint8_t)(tmp & 0xFF);
    } else {
        uint16_t result = (uint16_t)a - value - ((cpustatus & FLAG_CARRY) ? 0 : 1);
        if (result < 0x100) setcarry(); else clearcarry();
        zerocalc(result);
        signcalc(result);
        overflowcalc(result, a, (uint8_t)(~value));
        a = (uint8_t)(result & 0xFF);
    }
}

LYNX_RAMFUNC void cpu65sc02::op_sbc_imm()   { sbc_core(a, cpustatus, mem_read(pc++)); }
LYNX_RAMFUNC void cpu65sc02::op_sbc_zp()    { uint16_t addr=mem_read(pc++); sbc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_sbc_zpx()   { uint16_t addr=(mem_read(pc++)+x)&0xFF; sbc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_sbc_abs()   { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; sbc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_sbc_absx()  { uint16_t b=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+x; PAGE_CROSS_PENALTY(b,addr); sbc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_sbc_absy()  { uint16_t b=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+y; PAGE_CROSS_PENALTY(b,addr); sbc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_sbc_indx()  { uint16_t zp=(mem_read(pc++)+x)&0xFF; uint16_t addr=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); sbc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_sbc_indy()  { uint16_t zp=mem_read(pc++); uint16_t b=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); uint16_t addr=b+y; PAGE_CROSS_PENALTY(b,addr); sbc_core(a, cpustatus, mem_read(addr)); }
LYNX_RAMFUNC void cpu65sc02::op_sbc_zpind() { uint16_t zp=mem_read(pc++); uint16_t addr=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); sbc_core(a, cpustatus, mem_read(addr)); }

// ============================================================================
// AND / ORA / EOR
// ============================================================================
#define DEFINE_LOGIC_OP(NAME, OPER) \
LYNX_RAMFUNC void cpu65sc02::op_##NAME##_imm()  { a OPER##= mem_read(pc++); zerocalc(a); signcalc(a); } \
LYNX_RAMFUNC void cpu65sc02::op_##NAME##_zp()   { uint16_t addr=mem_read(pc++); a OPER##= mem_read(addr); zerocalc(a); signcalc(a); } \
LYNX_RAMFUNC void cpu65sc02::op_##NAME##_zpx()  { uint16_t addr=(mem_read(pc++)+x)&0xFF; a OPER##= mem_read(addr); zerocalc(a); signcalc(a); } \
LYNX_RAMFUNC void cpu65sc02::op_##NAME##_abs()  { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; a OPER##= mem_read(addr); zerocalc(a); signcalc(a); } \
LYNX_RAMFUNC void cpu65sc02::op_##NAME##_absx() { uint16_t b=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+x; PAGE_CROSS_PENALTY(b,addr); a OPER##= mem_read(addr); zerocalc(a); signcalc(a); } \
LYNX_RAMFUNC void cpu65sc02::op_##NAME##_absy() { uint16_t b=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+y; PAGE_CROSS_PENALTY(b,addr); a OPER##= mem_read(addr); zerocalc(a); signcalc(a); } \
LYNX_RAMFUNC void cpu65sc02::op_##NAME##_indx() { uint16_t zp=(mem_read(pc++)+x)&0xFF; uint16_t addr=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); a OPER##= mem_read(addr); zerocalc(a); signcalc(a); } \
LYNX_RAMFUNC void cpu65sc02::op_##NAME##_indy() { uint16_t zp=mem_read(pc++); uint16_t b=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); uint16_t addr=b+y; PAGE_CROSS_PENALTY(b,addr); a OPER##= mem_read(addr); zerocalc(a); signcalc(a); } \
LYNX_RAMFUNC void cpu65sc02::op_##NAME##_zpind(){ uint16_t zp=mem_read(pc++); uint16_t addr=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); a OPER##= mem_read(addr); zerocalc(a); signcalc(a); }

DEFINE_LOGIC_OP(and, &)
DEFINE_LOGIC_OP(ora, |)
DEFINE_LOGIC_OP(eor, ^)
#undef DEFINE_LOGIC_OP

// ============================================================================
// CMP / CPX / CPY
// ============================================================================
#define DEFINE_CMP(NAME, REG) \
LYNX_RAMFUNC void cpu65sc02::op_##NAME##_imm()  { uint8_t v=mem_read(pc++); uint16_t r=(uint16_t)REG-v; if(REG>=v) setcarry(); else clearcarry(); if(REG==v) setzero(); else clearzero(); signcalc(r); }

LYNX_RAMFUNC void cpu65sc02::op_cmp_imm()  { uint8_t v=mem_read(pc++); uint16_t r=(uint16_t)a-v; if(a>=v) setcarry(); else clearcarry(); if(a==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cmp_zp()   { uint16_t addr=mem_read(pc++); uint8_t v=mem_read(addr); uint16_t r=(uint16_t)a-v; if(a>=v) setcarry(); else clearcarry(); if(a==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cmp_zpx()  { uint16_t addr=(mem_read(pc++)+x)&0xFF; uint8_t v=mem_read(addr); uint16_t r=(uint16_t)a-v; if(a>=v) setcarry(); else clearcarry(); if(a==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cmp_abs()  { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint8_t v=mem_read(addr); uint16_t r=(uint16_t)a-v; if(a>=v) setcarry(); else clearcarry(); if(a==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cmp_absx() { uint16_t b=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+x; PAGE_CROSS_PENALTY(b,addr); uint8_t v=mem_read(addr); uint16_t r=(uint16_t)a-v; if(a>=v) setcarry(); else clearcarry(); if(a==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cmp_absy() { uint16_t b=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+y; PAGE_CROSS_PENALTY(b,addr); uint8_t v=mem_read(addr); uint16_t r=(uint16_t)a-v; if(a>=v) setcarry(); else clearcarry(); if(a==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cmp_indx() { uint16_t zp=(mem_read(pc++)+x)&0xFF; uint16_t addr=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); uint8_t v=mem_read(addr); uint16_t r=(uint16_t)a-v; if(a>=v) setcarry(); else clearcarry(); if(a==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cmp_indy() { uint16_t zp=mem_read(pc++); uint16_t b=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); uint16_t addr=b+y; PAGE_CROSS_PENALTY(b,addr); uint8_t v=mem_read(addr); uint16_t r=(uint16_t)a-v; if(a>=v) setcarry(); else clearcarry(); if(a==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cmp_zpind(){ uint16_t zp=mem_read(pc++); uint16_t addr=mem_read(zp)|(mem_read((zp+1)&0xFF)<<8); uint8_t v=mem_read(addr); uint16_t r=(uint16_t)a-v; if(a>=v) setcarry(); else clearcarry(); if(a==v) setzero(); else clearzero(); signcalc(r); }

LYNX_RAMFUNC void cpu65sc02::op_cpx_imm() { uint8_t v=mem_read(pc++); uint16_t r=(uint16_t)x-v; if(x>=v) setcarry(); else clearcarry(); if(x==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cpx_zp()  { uint16_t addr=mem_read(pc++); uint8_t v=mem_read(addr); uint16_t r=(uint16_t)x-v; if(x>=v) setcarry(); else clearcarry(); if(x==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cpx_abs() { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint8_t v=mem_read(addr); uint16_t r=(uint16_t)x-v; if(x>=v) setcarry(); else clearcarry(); if(x==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cpy_imm() { uint8_t v=mem_read(pc++); uint16_t r=(uint16_t)y-v; if(y>=v) setcarry(); else clearcarry(); if(y==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cpy_zp()  { uint16_t addr=mem_read(pc++); uint8_t v=mem_read(addr); uint16_t r=(uint16_t)y-v; if(y>=v) setcarry(); else clearcarry(); if(y==v) setzero(); else clearzero(); signcalc(r); }
LYNX_RAMFUNC void cpu65sc02::op_cpy_abs() { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint8_t v=mem_read(addr); uint16_t r=(uint16_t)y-v; if(y>=v) setcarry(); else clearcarry(); if(y==v) setzero(); else clearzero(); signcalc(r); }

// ============================================================================
// BIT (incluso il modo immediato, nuovo CMOS: in modo immediato tocca
// SOLO lo zero flag, non N/V) / TRB / TSB (nuovi CMOS)
// ============================================================================
LYNX_RAMFUNC void cpu65sc02::op_bit_zp()   { uint16_t addr=mem_read(pc++); uint8_t v=mem_read(addr); zerocalc((uint16_t)(a & v)); cpustatus=(cpustatus & 0x3F)|(v & 0xC0); }
LYNX_RAMFUNC void cpu65sc02::op_bit_zpx()  { uint16_t addr=(mem_read(pc++)+x)&0xFF; uint8_t v=mem_read(addr); zerocalc((uint16_t)(a & v)); cpustatus=(cpustatus & 0x3F)|(v & 0xC0); }
LYNX_RAMFUNC void cpu65sc02::op_bit_abs()  { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint8_t v=mem_read(addr); zerocalc((uint16_t)(a & v)); cpustatus=(cpustatus & 0x3F)|(v & 0xC0); }
LYNX_RAMFUNC void cpu65sc02::op_bit_absx() { uint16_t b=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint16_t addr=b+x; PAGE_CROSS_PENALTY(b,addr); uint8_t v=mem_read(addr); zerocalc((uint16_t)(a & v)); cpustatus=(cpustatus & 0x3F)|(v & 0xC0); }
LYNX_RAMFUNC void cpu65sc02::op_bit_imm()  { uint8_t v=mem_read(pc++); zerocalc((uint16_t)(a & v)); } // niente N/V in modo immediato

LYNX_RAMFUNC void cpu65sc02::op_trb_zp()  { uint16_t addr=mem_read(pc++); uint8_t v=mem_read(addr); zerocalc((uint16_t)(a & v)); mem_write(addr, v & ~a); }
LYNX_RAMFUNC void cpu65sc02::op_trb_abs() { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint8_t v=mem_read(addr); zerocalc((uint16_t)(a & v)); mem_write(addr, v & ~a); }
LYNX_RAMFUNC void cpu65sc02::op_tsb_zp()  { uint16_t addr=mem_read(pc++); uint8_t v=mem_read(addr); zerocalc((uint16_t)(a & v)); mem_write(addr, v | a); }
LYNX_RAMFUNC void cpu65sc02::op_tsb_abs() { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint8_t v=mem_read(addr); zerocalc((uint16_t)(a & v)); mem_write(addr, v | a); }

// ============================================================================
// Shift / Rotate / Inc / Dec (RMW), incluso INC A / DEC A nuovi CMOS
// ============================================================================
LYNX_RAMFUNC void cpu65sc02::op_asl_acc() { uint16_t r=(uint16_t)a<<1; carrycalc(r); zerocalc(r); signcalc(r); a=(uint8_t)r; }
LYNX_RAMFUNC void cpu65sc02::op_asl_zp()  { uint16_t addr=mem_read(pc++); uint8_t v=mem_read(addr); uint16_t r=v<<1; carrycalc(r); zerocalc(r); signcalc(r); mem_write(addr, r&0xFF); }
LYNX_RAMFUNC void cpu65sc02::op_asl_zpx() { uint16_t addr=(mem_read(pc++)+x)&0xFF; uint8_t v=mem_read(addr); uint16_t r=v<<1; carrycalc(r); zerocalc(r); signcalc(r); mem_write(addr, r&0xFF); }
LYNX_RAMFUNC void cpu65sc02::op_asl_abs() { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint8_t v=mem_read(addr); uint16_t r=v<<1; carrycalc(r); zerocalc(r); signcalc(r); mem_write(addr, r&0xFF); }
LYNX_RAMFUNC void cpu65sc02::op_asl_absx(){ uint16_t addr=(mem_read(pc)|(mem_read(pc+1)<<8))+x; pc+=2; uint8_t v=mem_read(addr); uint16_t r=v<<1; carrycalc(r); zerocalc(r); signcalc(r); mem_write(addr, r&0xFF); }

LYNX_RAMFUNC void cpu65sc02::op_lsr_acc() { if(a&1) setcarry(); else clearcarry(); a>>=1; zerocalc(a); clearsign(); }
LYNX_RAMFUNC void cpu65sc02::op_lsr_zp()  { uint16_t addr=mem_read(pc++); uint8_t v=mem_read(addr); if(v&1) setcarry(); else clearcarry(); uint8_t r=v>>1; zerocalc(r); clearsign(); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_lsr_zpx() { uint16_t addr=(mem_read(pc++)+x)&0xFF; uint8_t v=mem_read(addr); if(v&1) setcarry(); else clearcarry(); uint8_t r=v>>1; zerocalc(r); clearsign(); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_lsr_abs() { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint8_t v=mem_read(addr); if(v&1) setcarry(); else clearcarry(); uint8_t r=v>>1; zerocalc(r); clearsign(); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_lsr_absx(){ uint16_t addr=(mem_read(pc)|(mem_read(pc+1)<<8))+x; pc+=2; uint8_t v=mem_read(addr); if(v&1) setcarry(); else clearcarry(); uint8_t r=v>>1; zerocalc(r); clearsign(); mem_write(addr,r); }

LYNX_RAMFUNC void cpu65sc02::op_rol_acc() { uint16_t r=((uint16_t)a<<1)|(cpustatus&FLAG_CARRY); carrycalc(r); zerocalc(r); signcalc(r); a=(uint8_t)r; }
LYNX_RAMFUNC void cpu65sc02::op_rol_zp()  { uint16_t addr=mem_read(pc++); uint8_t v=mem_read(addr); uint16_t r=(v<<1)|(cpustatus&FLAG_CARRY); carrycalc(r); zerocalc(r); signcalc(r); mem_write(addr,r&0xFF); }
LYNX_RAMFUNC void cpu65sc02::op_rol_zpx() { uint16_t addr=(mem_read(pc++)+x)&0xFF; uint8_t v=mem_read(addr); uint16_t r=(v<<1)|(cpustatus&FLAG_CARRY); carrycalc(r); zerocalc(r); signcalc(r); mem_write(addr,r&0xFF); }
LYNX_RAMFUNC void cpu65sc02::op_rol_abs() { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint8_t v=mem_read(addr); uint16_t r=(v<<1)|(cpustatus&FLAG_CARRY); carrycalc(r); zerocalc(r); signcalc(r); mem_write(addr,r&0xFF); }
LYNX_RAMFUNC void cpu65sc02::op_rol_absx(){ uint16_t addr=(mem_read(pc)|(mem_read(pc+1)<<8))+x; pc+=2; uint8_t v=mem_read(addr); uint16_t r=(v<<1)|(cpustatus&FLAG_CARRY); carrycalc(r); zerocalc(r); signcalc(r); mem_write(addr,r&0xFF); }

LYNX_RAMFUNC void cpu65sc02::op_ror_acc() { uint8_t r=(a>>1)|((cpustatus&FLAG_CARRY)<<7); if(a&1) setcarry(); else clearcarry(); zerocalc(r); signcalc(r); a=r; }
LYNX_RAMFUNC void cpu65sc02::op_ror_zp()  { uint16_t addr=mem_read(pc++); uint8_t v=mem_read(addr); uint8_t r=(v>>1)|((cpustatus&FLAG_CARRY)<<7); if(v&1) setcarry(); else clearcarry(); zerocalc(r); signcalc(r); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_ror_zpx() { uint16_t addr=(mem_read(pc++)+x)&0xFF; uint8_t v=mem_read(addr); uint8_t r=(v>>1)|((cpustatus&FLAG_CARRY)<<7); if(v&1) setcarry(); else clearcarry(); zerocalc(r); signcalc(r); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_ror_abs() { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint8_t v=mem_read(addr); uint8_t r=(v>>1)|((cpustatus&FLAG_CARRY)<<7); if(v&1) setcarry(); else clearcarry(); zerocalc(r); signcalc(r); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_ror_absx(){ uint16_t addr=(mem_read(pc)|(mem_read(pc+1)<<8))+x; pc+=2; uint8_t v=mem_read(addr); uint8_t r=(v>>1)|((cpustatus&FLAG_CARRY)<<7); if(v&1) setcarry(); else clearcarry(); zerocalc(r); signcalc(r); mem_write(addr,r); }

LYNX_RAMFUNC void cpu65sc02::op_inc_zp()  { uint16_t addr=mem_read(pc++); uint8_t r=mem_read(addr)+1; zerocalc(r); signcalc(r); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_inc_zpx() { uint16_t addr=(mem_read(pc++)+x)&0xFF; uint8_t r=mem_read(addr)+1; zerocalc(r); signcalc(r); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_inc_abs() { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint8_t r=mem_read(addr)+1; zerocalc(r); signcalc(r); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_inc_absx(){ uint16_t addr=(mem_read(pc)|(mem_read(pc+1)<<8))+x; pc+=2; uint8_t r=mem_read(addr)+1; zerocalc(r); signcalc(r); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_inc_acc() { a++; zerocalc(a); signcalc(a); }

LYNX_RAMFUNC void cpu65sc02::op_dec_zp()  { uint16_t addr=mem_read(pc++); uint8_t r=mem_read(addr)-1; zerocalc(r); signcalc(r); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_dec_zpx() { uint16_t addr=(mem_read(pc++)+x)&0xFF; uint8_t r=mem_read(addr)-1; zerocalc(r); signcalc(r); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_dec_abs() { uint16_t addr=mem_read(pc)|(mem_read(pc+1)<<8); pc+=2; uint8_t r=mem_read(addr)-1; zerocalc(r); signcalc(r); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_dec_absx(){ uint16_t addr=(mem_read(pc)|(mem_read(pc+1)<<8))+x; pc+=2; uint8_t r=mem_read(addr)-1; zerocalc(r); signcalc(r); mem_write(addr,r); }
LYNX_RAMFUNC void cpu65sc02::op_dec_acc() { a--; zerocalc(a); signcalc(a); }

LYNX_RAMFUNC void cpu65sc02::op_inx() { x++; zerocalc(x); signcalc(x); }
LYNX_RAMFUNC void cpu65sc02::op_iny() { y++; zerocalc(y); signcalc(y); }
LYNX_RAMFUNC void cpu65sc02::op_dex() { x--; zerocalc(x); signcalc(x); }
LYNX_RAMFUNC void cpu65sc02::op_dey() { y--; zerocalc(y); signcalc(y); }

// ============================================================================
// Branch (incluso BRA, nuovo CMOS: sempre preso)
// ============================================================================
#define DEFINE_BRANCH(NAME, COND) \
LYNX_RAMFUNC void cpu65sc02::op_##NAME() { \
    int8_t off=(int8_t)mem_read(pc++); \
    if (COND) { uint16_t old=pc; pc+=off; extraCyclesAccum += ((old&0xFF00)!=(pc&0xFF00))?2:1; } \
}

DEFINE_BRANCH(bpl, (cpustatus & FLAG_SIGN)==0)
DEFINE_BRANCH(bmi, (cpustatus & FLAG_SIGN)!=0)
DEFINE_BRANCH(bvc, (cpustatus & FLAG_OVERFLOW)==0)
DEFINE_BRANCH(bvs, (cpustatus & FLAG_OVERFLOW)!=0)
DEFINE_BRANCH(bcc, (cpustatus & FLAG_CARRY)==0)
DEFINE_BRANCH(bcs, (cpustatus & FLAG_CARRY)!=0)
DEFINE_BRANCH(bne, (cpustatus & FLAG_ZERO)==0)
DEFINE_BRANCH(beq, (cpustatus & FLAG_ZERO)!=0)
#undef DEFINE_BRANCH

LYNX_RAMFUNC void cpu65sc02::op_bra() {
    int8_t off=(int8_t)mem_read(pc++);
    uint16_t old=pc; pc+=off;
    extraCyclesAccum += ((old&0xFF00)!=(pc&0xFF00)) ? 1 : 0; // base gia' include il "preso"
}

// ============================================================================
// Jump / Call
// ============================================================================
LYNX_RAMFUNC void cpu65sc02::op_jmp_abs() { pc = mem_read(pc) | (mem_read(pc+1)<<8); }

// FIX rispetto all'NMOS: niente wraparound di pagina sull'indirect fetch.
LYNX_RAMFUNC void cpu65sc02::op_jmp_ind() {
    uint16_t addr = mem_read(pc) | (mem_read(pc+1)<<8);
    uint16_t addr2 = addr + 1; // sul 65C02 NON wrappa dentro la stessa pagina
    pc = mem_read(addr) | (mem_read(addr2)<<8);
}

// Nuovo indirizzamento CMOS: JMP (abs,X)
LYNX_RAMFUNC void cpu65sc02::op_jmp_indx() {
    uint16_t base = mem_read(pc) | (mem_read(pc+1)<<8);
    uint16_t addr = base + x;
    pc = mem_read(addr) | (mem_read((uint16_t)(addr+1))<<8);
}

LYNX_RAMFUNC void cpu65sc02::op_jsr() {
    uint16_t addr = mem_read(pc) | (mem_read(pc+1)<<8);
    push16(pc+1);
    pc = addr;
}
LYNX_RAMFUNC void cpu65sc02::op_rts() { pc = pull16() + 1; }
LYNX_RAMFUNC void cpu65sc02::op_rti() { cpustatus = pull8() | FLAG_CONSTANT; pc = pull16(); }

LYNX_RAMFUNC void cpu65sc02::op_brk() {
    pc++;
    push16(pc);
    push8(cpustatus | FLAG_BREAK);
    setinterrupt();
    cleardecimal(); // comportamento CMOS
    pc = mem_read(0xFFFE) | (mem_read(0xFFFF)<<8);
}

// ============================================================================
// Stack (incluso PHX/PLX/PHY/PLY, nuovi CMOS)
// ============================================================================
LYNX_RAMFUNC void cpu65sc02::op_pha() { push8(a); }
LYNX_RAMFUNC void cpu65sc02::op_pla() { a = pull8(); zerocalc(a); signcalc(a); }
LYNX_RAMFUNC void cpu65sc02::op_php() { push8(cpustatus | FLAG_BREAK); }
LYNX_RAMFUNC void cpu65sc02::op_plp() { cpustatus = pull8() | FLAG_CONSTANT; }
LYNX_RAMFUNC void cpu65sc02::op_phx() { push8(x); }
LYNX_RAMFUNC void cpu65sc02::op_plx() { x = pull8(); zerocalc(x); signcalc(x); }
LYNX_RAMFUNC void cpu65sc02::op_phy() { push8(y); }
LYNX_RAMFUNC void cpu65sc02::op_ply() { y = pull8(); zerocalc(y); signcalc(y); }

// ============================================================================
// Transfer / Flag
// ============================================================================
LYNX_RAMFUNC void cpu65sc02::op_tax() { x=a; zerocalc(x); signcalc(x); }
LYNX_RAMFUNC void cpu65sc02::op_txa() { a=x; zerocalc(a); signcalc(a); }
LYNX_RAMFUNC void cpu65sc02::op_tay() { y=a; zerocalc(y); signcalc(y); }
LYNX_RAMFUNC void cpu65sc02::op_tya() { a=y; zerocalc(a); signcalc(a); }
LYNX_RAMFUNC void cpu65sc02::op_tsx() { x=sp; zerocalc(x); signcalc(x); }
LYNX_RAMFUNC void cpu65sc02::op_txs() { sp=x; }

LYNX_RAMFUNC void cpu65sc02::op_clc() { clearcarry(); }
LYNX_RAMFUNC void cpu65sc02::op_sec() { setcarry(); }
LYNX_RAMFUNC void cpu65sc02::op_cli() { clearinterrupt(); }
LYNX_RAMFUNC void cpu65sc02::op_sei() { setinterrupt(); }
LYNX_RAMFUNC void cpu65sc02::op_clv() { clearoverflow(); }
LYNX_RAMFUNC void cpu65sc02::op_cld() { cleardecimal(); }
LYNX_RAMFUNC void cpu65sc02::op_sed() { setdecimal(); }
LYNX_RAMFUNC void cpu65sc02::op_nop() {}

// NOP "puri": a differenza del 6510 (dove leggere l'operando di un
// opcode illegale poteva avere side-effect su I/O), sul 65C02 questi
// slot sono NOP veri: avanzano il PC ma NON eseguono la lettura.
LYNX_RAMFUNC void cpu65sc02::op_nop_imm()  { pc++; }
LYNX_RAMFUNC void cpu65sc02::op_nop_zp()   { pc++; }
LYNX_RAMFUNC void cpu65sc02::op_nop_zpx()  { pc++; }
LYNX_RAMFUNC void cpu65sc02::op_nop_abs()  { pc+=2; }
LYNX_RAMFUNC void cpu65sc02::op_nop_absx() { pc+=2; }

// ============================================================================
// Dispatch table
// ============================================================================
void cpu65sc02::initOpcodeTable() {
    for (int i=0;i<256;i++) opcodeTable[i] = &cpu65sc02::op_nop;

    opcodeTable[0x00]=&cpu65sc02::op_brk;
    opcodeTable[0x01]=&cpu65sc02::op_ora_indx;
    opcodeTable[0x02]=&cpu65sc02::op_nop_imm;
    opcodeTable[0x04]=&cpu65sc02::op_tsb_zp;
    opcodeTable[0x05]=&cpu65sc02::op_ora_zp;
    opcodeTable[0x06]=&cpu65sc02::op_asl_zp;
    opcodeTable[0x08]=&cpu65sc02::op_php;
    opcodeTable[0x09]=&cpu65sc02::op_ora_imm;
    opcodeTable[0x0A]=&cpu65sc02::op_asl_acc;
    opcodeTable[0x0C]=&cpu65sc02::op_tsb_abs;
    opcodeTable[0x0D]=&cpu65sc02::op_ora_abs;
    opcodeTable[0x0E]=&cpu65sc02::op_asl_abs;

    opcodeTable[0x10]=&cpu65sc02::op_bpl;
    opcodeTable[0x11]=&cpu65sc02::op_ora_indy;
    opcodeTable[0x12]=&cpu65sc02::op_ora_zpind;
    opcodeTable[0x14]=&cpu65sc02::op_trb_zp;
    opcodeTable[0x15]=&cpu65sc02::op_ora_zpx;
    opcodeTable[0x16]=&cpu65sc02::op_asl_zpx;
    opcodeTable[0x18]=&cpu65sc02::op_clc;
    opcodeTable[0x19]=&cpu65sc02::op_ora_absy;
    opcodeTable[0x1A]=&cpu65sc02::op_inc_acc;
    opcodeTable[0x1C]=&cpu65sc02::op_trb_abs;
    opcodeTable[0x1D]=&cpu65sc02::op_ora_absx;
    opcodeTable[0x1E]=&cpu65sc02::op_asl_absx;

    opcodeTable[0x20]=&cpu65sc02::op_jsr;
    opcodeTable[0x21]=&cpu65sc02::op_and_indx;
    opcodeTable[0x22]=&cpu65sc02::op_nop_imm;
    opcodeTable[0x24]=&cpu65sc02::op_bit_zp;
    opcodeTable[0x25]=&cpu65sc02::op_and_zp;
    opcodeTable[0x26]=&cpu65sc02::op_rol_zp;
    opcodeTable[0x28]=&cpu65sc02::op_plp;
    opcodeTable[0x29]=&cpu65sc02::op_and_imm;
    opcodeTable[0x2A]=&cpu65sc02::op_rol_acc;
    opcodeTable[0x2C]=&cpu65sc02::op_bit_abs;
    opcodeTable[0x2D]=&cpu65sc02::op_and_abs;
    opcodeTable[0x2E]=&cpu65sc02::op_rol_abs;

    opcodeTable[0x30]=&cpu65sc02::op_bmi;
    opcodeTable[0x31]=&cpu65sc02::op_and_indy;
    opcodeTable[0x32]=&cpu65sc02::op_and_zpind;
    opcodeTable[0x34]=&cpu65sc02::op_bit_zpx;
    opcodeTable[0x35]=&cpu65sc02::op_and_zpx;
    opcodeTable[0x36]=&cpu65sc02::op_rol_zpx;
    opcodeTable[0x38]=&cpu65sc02::op_sec;
    opcodeTable[0x39]=&cpu65sc02::op_and_absy;
    opcodeTable[0x3A]=&cpu65sc02::op_dec_acc;
    opcodeTable[0x3C]=&cpu65sc02::op_bit_absx;
    opcodeTable[0x3D]=&cpu65sc02::op_and_absx;
    opcodeTable[0x3E]=&cpu65sc02::op_rol_absx;

    opcodeTable[0x40]=&cpu65sc02::op_rti;
    opcodeTable[0x41]=&cpu65sc02::op_eor_indx;
    opcodeTable[0x42]=&cpu65sc02::op_nop_imm;
    opcodeTable[0x44]=&cpu65sc02::op_nop_zp;
    opcodeTable[0x45]=&cpu65sc02::op_eor_zp;
    opcodeTable[0x46]=&cpu65sc02::op_lsr_zp;
    opcodeTable[0x48]=&cpu65sc02::op_pha;
    opcodeTable[0x49]=&cpu65sc02::op_eor_imm;
    opcodeTable[0x4A]=&cpu65sc02::op_lsr_acc;
    opcodeTable[0x4C]=&cpu65sc02::op_jmp_abs;
    opcodeTable[0x4D]=&cpu65sc02::op_eor_abs;
    opcodeTable[0x4E]=&cpu65sc02::op_lsr_abs;

    opcodeTable[0x50]=&cpu65sc02::op_bvc;
    opcodeTable[0x51]=&cpu65sc02::op_eor_indy;
    opcodeTable[0x52]=&cpu65sc02::op_eor_zpind;
    opcodeTable[0x54]=&cpu65sc02::op_nop_zpx;
    opcodeTable[0x55]=&cpu65sc02::op_eor_zpx;
    opcodeTable[0x56]=&cpu65sc02::op_lsr_zpx;
    opcodeTable[0x58]=&cpu65sc02::op_cli;
    opcodeTable[0x59]=&cpu65sc02::op_eor_absy;
    opcodeTable[0x5A]=&cpu65sc02::op_phy;
    opcodeTable[0x5C]=&cpu65sc02::op_nop_abs;
    opcodeTable[0x5D]=&cpu65sc02::op_eor_absx;
    opcodeTable[0x5E]=&cpu65sc02::op_lsr_absx;

    opcodeTable[0x60]=&cpu65sc02::op_rts;
    opcodeTable[0x61]=&cpu65sc02::op_adc_indx;
    opcodeTable[0x62]=&cpu65sc02::op_nop_imm;
    opcodeTable[0x64]=&cpu65sc02::op_stz_zp;
    opcodeTable[0x65]=&cpu65sc02::op_adc_zp;
    opcodeTable[0x66]=&cpu65sc02::op_ror_zp;
    opcodeTable[0x68]=&cpu65sc02::op_pla;
    opcodeTable[0x69]=&cpu65sc02::op_adc_imm;
    opcodeTable[0x6A]=&cpu65sc02::op_ror_acc;
    opcodeTable[0x6C]=&cpu65sc02::op_jmp_ind;
    opcodeTable[0x6D]=&cpu65sc02::op_adc_abs;
    opcodeTable[0x6E]=&cpu65sc02::op_ror_abs;

    opcodeTable[0x70]=&cpu65sc02::op_bvs;
    opcodeTable[0x71]=&cpu65sc02::op_adc_indy;
    opcodeTable[0x72]=&cpu65sc02::op_adc_zpind;
    opcodeTable[0x74]=&cpu65sc02::op_stz_zpx;
    opcodeTable[0x75]=&cpu65sc02::op_adc_zpx;
    opcodeTable[0x76]=&cpu65sc02::op_ror_zpx;
    opcodeTable[0x78]=&cpu65sc02::op_sei;
    opcodeTable[0x79]=&cpu65sc02::op_adc_absy;
    opcodeTable[0x7A]=&cpu65sc02::op_ply;
    opcodeTable[0x7C]=&cpu65sc02::op_jmp_indx;
    opcodeTable[0x7D]=&cpu65sc02::op_adc_absx;
    opcodeTable[0x7E]=&cpu65sc02::op_ror_absx;

    opcodeTable[0x80]=&cpu65sc02::op_bra;
    opcodeTable[0x81]=&cpu65sc02::op_sta_indx;
    opcodeTable[0x82]=&cpu65sc02::op_nop_imm;
    opcodeTable[0x84]=&cpu65sc02::op_sty_zp;
    opcodeTable[0x85]=&cpu65sc02::op_sta_zp;
    opcodeTable[0x86]=&cpu65sc02::op_stx_zp;
    opcodeTable[0x88]=&cpu65sc02::op_dey;
    opcodeTable[0x89]=&cpu65sc02::op_bit_imm;
    opcodeTable[0x8A]=&cpu65sc02::op_txa;
    opcodeTable[0x8C]=&cpu65sc02::op_sty_abs;
    opcodeTable[0x8D]=&cpu65sc02::op_sta_abs;
    opcodeTable[0x8E]=&cpu65sc02::op_stx_abs;

    opcodeTable[0x90]=&cpu65sc02::op_bcc;
    opcodeTable[0x91]=&cpu65sc02::op_sta_indy;
    opcodeTable[0x92]=&cpu65sc02::op_sta_zpind;
    opcodeTable[0x94]=&cpu65sc02::op_sty_zpx;
    opcodeTable[0x95]=&cpu65sc02::op_sta_zpx;
    opcodeTable[0x96]=&cpu65sc02::op_stx_zpy;
    opcodeTable[0x98]=&cpu65sc02::op_tya;
    opcodeTable[0x99]=&cpu65sc02::op_sta_absy;
    opcodeTable[0x9A]=&cpu65sc02::op_txs;
    opcodeTable[0x9C]=&cpu65sc02::op_stz_abs;
    opcodeTable[0x9D]=&cpu65sc02::op_sta_absx;
    opcodeTable[0x9E]=&cpu65sc02::op_stz_absx;

    opcodeTable[0xA0]=&cpu65sc02::op_ldy_imm;
    opcodeTable[0xA1]=&cpu65sc02::op_lda_indx;
    opcodeTable[0xA2]=&cpu65sc02::op_ldx_imm;
    opcodeTable[0xA4]=&cpu65sc02::op_ldy_zp;
    opcodeTable[0xA5]=&cpu65sc02::op_lda_zp;
    opcodeTable[0xA6]=&cpu65sc02::op_ldx_zp;
    opcodeTable[0xA8]=&cpu65sc02::op_tay;
    opcodeTable[0xA9]=&cpu65sc02::op_lda_imm;
    opcodeTable[0xAA]=&cpu65sc02::op_tax;
    opcodeTable[0xAC]=&cpu65sc02::op_ldy_abs;
    opcodeTable[0xAD]=&cpu65sc02::op_lda_abs;
    opcodeTable[0xAE]=&cpu65sc02::op_ldx_abs;

    opcodeTable[0xB0]=&cpu65sc02::op_bcs;
    opcodeTable[0xB1]=&cpu65sc02::op_lda_indy;
    opcodeTable[0xB2]=&cpu65sc02::op_lda_zpind;
    opcodeTable[0xB4]=&cpu65sc02::op_ldy_zpx;
    opcodeTable[0xB5]=&cpu65sc02::op_lda_zpx;
    opcodeTable[0xB6]=&cpu65sc02::op_ldx_zpy;
    opcodeTable[0xB8]=&cpu65sc02::op_clv;
    opcodeTable[0xB9]=&cpu65sc02::op_lda_absy;
    opcodeTable[0xBA]=&cpu65sc02::op_tsx;
    opcodeTable[0xBC]=&cpu65sc02::op_ldy_absx;
    opcodeTable[0xBD]=&cpu65sc02::op_lda_absx;
    opcodeTable[0xBE]=&cpu65sc02::op_ldx_absy;

    opcodeTable[0xC0]=&cpu65sc02::op_cpy_imm;
    opcodeTable[0xC1]=&cpu65sc02::op_cmp_indx;
    opcodeTable[0xC2]=&cpu65sc02::op_nop_imm;
    opcodeTable[0xC4]=&cpu65sc02::op_cpy_zp;
    opcodeTable[0xC5]=&cpu65sc02::op_cmp_zp;
    opcodeTable[0xC6]=&cpu65sc02::op_dec_zp;
    opcodeTable[0xC8]=&cpu65sc02::op_iny;
    opcodeTable[0xC9]=&cpu65sc02::op_cmp_imm;
    opcodeTable[0xCA]=&cpu65sc02::op_dex;
    opcodeTable[0xCB]=&cpu65sc02::op_nop; // WAI: presenza incerta sul core Lynx
    opcodeTable[0xCC]=&cpu65sc02::op_cpy_abs;
    opcodeTable[0xCD]=&cpu65sc02::op_cmp_abs;
    opcodeTable[0xCE]=&cpu65sc02::op_dec_abs;

    opcodeTable[0xD0]=&cpu65sc02::op_bne;
    opcodeTable[0xD1]=&cpu65sc02::op_cmp_indy;
    opcodeTable[0xD2]=&cpu65sc02::op_cmp_zpind;
    opcodeTable[0xD4]=&cpu65sc02::op_nop_zpx;
    opcodeTable[0xD5]=&cpu65sc02::op_cmp_zpx;
    opcodeTable[0xD6]=&cpu65sc02::op_dec_zpx;
    opcodeTable[0xD8]=&cpu65sc02::op_cld;
    opcodeTable[0xD9]=&cpu65sc02::op_cmp_absy;
    opcodeTable[0xDA]=&cpu65sc02::op_phx;
    opcodeTable[0xDB]=&cpu65sc02::op_nop; // STP: presenza incerta
    opcodeTable[0xDC]=&cpu65sc02::op_nop_absx;
    opcodeTable[0xDD]=&cpu65sc02::op_cmp_absx;
    opcodeTable[0xDE]=&cpu65sc02::op_dec_absx;

    opcodeTable[0xE0]=&cpu65sc02::op_cpx_imm;
    opcodeTable[0xE1]=&cpu65sc02::op_sbc_indx;
    opcodeTable[0xE2]=&cpu65sc02::op_nop_imm;
    opcodeTable[0xE4]=&cpu65sc02::op_cpx_zp;
    opcodeTable[0xE5]=&cpu65sc02::op_sbc_zp;
    opcodeTable[0xE6]=&cpu65sc02::op_inc_zp;
    opcodeTable[0xE8]=&cpu65sc02::op_inx;
    opcodeTable[0xE9]=&cpu65sc02::op_sbc_imm;
    opcodeTable[0xEA]=&cpu65sc02::op_nop;
    opcodeTable[0xEC]=&cpu65sc02::op_cpx_abs;
    opcodeTable[0xED]=&cpu65sc02::op_sbc_abs;
    opcodeTable[0xEE]=&cpu65sc02::op_inc_abs;

    opcodeTable[0xF0]=&cpu65sc02::op_beq;
    opcodeTable[0xF1]=&cpu65sc02::op_sbc_indy;
    opcodeTable[0xF2]=&cpu65sc02::op_sbc_zpind;
    opcodeTable[0xF4]=&cpu65sc02::op_nop_zpx;
    opcodeTable[0xF5]=&cpu65sc02::op_sbc_zpx;
    opcodeTable[0xF6]=&cpu65sc02::op_inc_zpx;
    opcodeTable[0xF8]=&cpu65sc02::op_sed;
    opcodeTable[0xF9]=&cpu65sc02::op_sbc_absy;
    opcodeTable[0xFA]=&cpu65sc02::op_plx;
    opcodeTable[0xFC]=&cpu65sc02::op_nop_absx;
    opcodeTable[0xFD]=&cpu65sc02::op_sbc_absx;
    opcodeTable[0xFE]=&cpu65sc02::op_inc_absx;

    // Tutti gli slot rimasti a op_nop di default sono gli ex RMBx/SMBx/
    // BBRx/BBSx (assenti sul 65SC02) e i NOP a 1 byte del 65C02.
}
