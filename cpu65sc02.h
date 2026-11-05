/*
 * Core CPU 65SC02 per l'emulatore Atari Lynx.
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 *
 * Adattato dal core 6510 (NMOS) del progetto Pi64 (C64) per riflettere
 * il processore realmente montato nel chip Mikey del Lynx: un 65SC02,
 * cioe' la variante CMOS del WDC 65C02 PRIVA delle istruzioni di
 * bit-manipulation (RMBx/SMBx/BBRx/BBSx). 
 *
 */

#ifndef CPU65SC02_H
#define CPU65SC02_H

#include <stdint.h>
#include <cstddef>
#include <cstring>

#define RAMSIZE_LYNX 65536

#define FLAG_CARRY     0x01
#define FLAG_ZERO      0x02
#define FLAG_INTERRUPT 0x04
#define FLAG_DECIMAL   0x08
#define FLAG_BREAK     0x10
#define FLAG_CONSTANT  0x20
#define FLAG_OVERFLOW  0x40
#define FLAG_SIGN      0x80
#define BASE_STACK     0x100

// Firma degli hook verso Suzy/Mikey. Verranno implementati come
// classi vere e proprie piu' avanti; per ora sono semplici puntatori
// a funzione (o funzioni membro di un "bus" che il chiamante puo'
// registrare), cosi' la CPU non deve conoscere nulla della loro
// implementazione interna.
typedef uint8_t (*MemReadHook)(void* ctx, uint16_t addr);
typedef void (*MemWriteHook)(void* ctx, uint16_t addr, uint8_t value);
typedef void (*CartBankHook)(void* ctx, uint8_t value);

class cpu65sc02 {
public:
    cpu65sc02();

    void resetCPU();
    int execCPU(int tickcount = 1);
    void nmi();
    void irq();

    uint16_t getpc() const { return pc; }
    uint8_t getop() const { return opcode; }

    // Getter di debug (utili per test unitari e per un futuro debugger/
    // sistema di savestate). Non usati dal core stesso.
    uint8_t getA() const { return a; }
    uint8_t getX() const { return x; }
    uint8_t getY() const { return y; }
    uint8_t getSP() const { return sp; }
    uint8_t getStatus() const { return cpustatus; }

    // Registra gli hook per Suzy (0xFC00-0xFCFF) e Mikey (0xFD00-0xFDFF).
    // Se non registrati, quegli indirizzi si comportano come RAM normale
    // (utile per testare la CPU da sola prima di avere Suzy/Mikey pronti).
    void attachSuzy(void* ctx, MemReadHook rd, MemWriteHook wr);
    void attachMikey(void* ctx, MemReadHook rd, MemWriteHook wr);

    // Punto di ingresso pubblico, usato anche da eventuali strumenti di
    // debug/disassembly esterni.
    inline uint8_t mem_read(uint16_t addr) { return mem_read_fast(addr); }
    inline void mem_write(uint16_t addr, uint8_t val) { mem_write_fast(addr, val); }

    uint8_t memory[RAMSIZE_LYNX];

    // Carica l'immagine della boot ROM (512 byte, 0xFE00-0xFFFF) in un
    // buffer SEPARATO dalla RAM. Va chiamato al posto del vecchio
    // memcpy(cpu.memory + 0xFE00, ...).
    inline void loadBootRom(const uint8_t* data, size_t len) {
        size_t n = len > sizeof(bootRom) ? sizeof(bootRom) : len;
        memcpy(bootRom, data, n);
        if (n < sizeof(bootRom)) memset(bootRom + n, 0xFF, sizeof(bootRom) - n);
    }

    void sleepCycles(uint32_t cycles) { sleepCyclesRemaining += cycles; }

    void Halt(bool halt) { halted = halt; }
    bool IsHalted() const { return halted; }

private:
    typedef void (cpu65sc02::*OpcodeHandler)();
    static OpcodeHandler opcodeTable[256];
    static bool opcodeTableInitialized;
    static void initOpcodeTable();

    bool halted = false;

    uint16_t pc;
    uint8_t sp, a, x, y, cpustatus;
    uint8_t opcode;

    void* suzyCtx = nullptr;
    MemReadHook suzyRead = nullptr;
    MemWriteHook suzyWrite = nullptr;

    void* mikeyCtx = nullptr;
    MemReadHook mikeyRead = nullptr;
    MemWriteHook mikeyWrite = nullptr;

    inline void push16(uint16_t v) {
        mem_write(BASE_STACK + sp, (v >> 8) & 0xFF);
        mem_write(BASE_STACK + ((sp - 1) & 0xFF), v & 0xFF);
        sp -= 2;
    }
    inline void push8(uint8_t v) { mem_write(BASE_STACK + sp--, v); }
    inline uint16_t pull16() {
        uint16_t t = mem_read(BASE_STACK + ((sp + 1) & 0xFF)) |
                     ((uint16_t)mem_read(BASE_STACK + ((sp + 2) & 0xFF)) << 8);
        sp += 2;
        return t;
    }
    inline uint8_t pull8() { return mem_read(BASE_STACK + ++sp); }

    // ===== Load/Store =====
    void op_lda_imm(); void op_lda_zp(); void op_lda_zpx(); void op_lda_abs();
    void op_lda_absx(); void op_lda_absy(); void op_lda_indx(); void op_lda_indy();
    void op_lda_zpind(); // (zp) - nuovo modo CMOS

    void op_ldx_imm(); void op_ldx_zp(); void op_ldx_zpy(); void op_ldx_abs(); void op_ldx_absy();
    void op_ldy_imm(); void op_ldy_zp(); void op_ldy_zpx(); void op_ldy_abs(); void op_ldy_absx();

    void op_sta_zp(); void op_sta_zpx(); void op_sta_abs(); void op_sta_absx();
    void op_sta_absy(); void op_sta_indx(); void op_sta_indy(); void op_sta_zpind();

    void op_stx_zp(); void op_stx_zpy(); void op_stx_abs();
    void op_sty_zp(); void op_sty_zpx(); void op_sty_abs();

    void op_stz_zp(); void op_stz_zpx(); void op_stz_abs(); void op_stz_absx(); // nuovo CMOS

    // ===== ALU =====
    void op_adc_imm(); void op_adc_zp(); void op_adc_zpx(); void op_adc_abs();
    void op_adc_absx(); void op_adc_absy(); void op_adc_indx(); void op_adc_indy(); void op_adc_zpind();

    void op_sbc_imm(); void op_sbc_zp(); void op_sbc_zpx(); void op_sbc_abs();
    void op_sbc_absx(); void op_sbc_absy(); void op_sbc_indx(); void op_sbc_indy(); void op_sbc_zpind();

    void op_and_imm(); void op_and_zp(); void op_and_zpx(); void op_and_abs();
    void op_and_absx(); void op_and_absy(); void op_and_indx(); void op_and_indy(); void op_and_zpind();

    void op_ora_imm(); void op_ora_zp(); void op_ora_zpx(); void op_ora_abs();
    void op_ora_absx(); void op_ora_absy(); void op_ora_indx(); void op_ora_indy(); void op_ora_zpind();

    void op_eor_imm(); void op_eor_zp(); void op_eor_zpx(); void op_eor_abs();
    void op_eor_absx(); void op_eor_absy(); void op_eor_indx(); void op_eor_indy(); void op_eor_zpind();

    void op_cmp_imm(); void op_cmp_zp(); void op_cmp_zpx(); void op_cmp_abs();
    void op_cmp_absx(); void op_cmp_absy(); void op_cmp_indx(); void op_cmp_indy(); void op_cmp_zpind();

    void op_cpx_imm(); void op_cpx_zp(); void op_cpx_abs();
    void op_cpy_imm(); void op_cpy_zp(); void op_cpy_abs();

    void op_bit_zp(); void op_bit_abs(); void op_bit_zpx(); void op_bit_absx(); void op_bit_imm(); // imm nuovo CMOS

    void op_trb_zp(); void op_trb_abs(); // nuovo CMOS
    void op_tsb_zp(); void op_tsb_abs(); // nuovo CMOS

    // ===== Shift/Rotate/Inc/Dec (RMW) =====
    void op_asl_acc(); void op_asl_zp(); void op_asl_zpx(); void op_asl_abs(); void op_asl_absx();
    void op_lsr_acc(); void op_lsr_zp(); void op_lsr_zpx(); void op_lsr_abs(); void op_lsr_absx();
    void op_rol_acc(); void op_rol_zp(); void op_rol_zpx(); void op_rol_abs(); void op_rol_absx();
    void op_ror_acc(); void op_ror_zp(); void op_ror_zpx(); void op_ror_abs(); void op_ror_absx();

    void op_inc_zp(); void op_inc_zpx(); void op_inc_abs(); void op_inc_absx(); void op_inc_acc(); // acc nuovo CMOS
    void op_dec_zp(); void op_dec_zpx(); void op_dec_abs(); void op_dec_absx(); void op_dec_acc(); // acc nuovo CMOS
    void op_inx(); void op_iny(); void op_dex(); void op_dey();

    // ===== Branch =====
    void op_bpl(); void op_bmi(); void op_bvc(); void op_bvs();
    void op_bcc(); void op_bcs(); void op_bne(); void op_beq();
    void op_bra(); // nuovo CMOS: branch incondizionato

    // ===== Jump/Call =====
    void op_jmp_abs(); void op_jmp_ind(); void op_jmp_indx(); // indx = JMP (abs,X), nuovo CMOS
    void op_jsr(); void op_rts(); void op_rti(); void op_brk();

    // ===== Stack =====
    void op_pha(); void op_pla(); void op_php(); void op_plp();
    void op_phx(); void op_plx(); void op_phy(); void op_ply(); // nuovi CMOS

    // ===== Transfer/Flag =====
    void op_tax(); void op_txa(); void op_tay(); void op_tya(); void op_tsx(); void op_txs();
    void op_clc(); void op_sec(); void op_cli(); void op_sei();
    void op_clv(); void op_cld(); void op_sed(); void op_nop();

    // NOP con vari indirizzamenti (per gli slot ex-illegali del 6502
    // NMOS che sul 65C02 diventano NOP "veri" a lunghezza fissa, senza
    // side-effect di lettura: a differenza del 6510 qui non leggono
    // davvero l'operando).
    void op_nop_imm(); void op_nop_zp(); void op_nop_zpx();
    void op_nop_abs(); void op_nop_absx();
    
    inline uint8_t mem_read_fast(uint16_t addr) {
        // FAST PATH: Se è RAM normale (0x0000 - 0xFBFF), leggi subito senza altri check
        if (addr < 0xFC00) return memory[addr];
        
        if (addr == 0xFFF9) return mapctl;

        uint8_t page = addr >> 8;
        if (page == 0xFC) { // Suzy
            if (!(mapctl & 0x01) && suzyRead) return suzyRead(suzyCtx, addr);
            return memory[addr];
        }
        if (page == 0xFD) { // Mikey
            if (!(mapctl & 0x02) && mikeyRead) return mikeyRead(mikeyCtx, addr);
            return memory[addr];
        }
        
        // ROM space
        if (addr >= 0xFE00) {
            if (!(mapctl & 0x04) && addr <= 0xFFF7) return bootRom[addr - 0xFE00];
            if (!(mapctl & 0x08) && addr >= 0xFFFA) return bootRom[addr - 0xFE00];
            return memory[addr];
        }
        
        return memory[addr];
    }

    inline void mem_write_fast(uint16_t addr, uint8_t value) {
        // FAST PATH: Scrittura in RAM normale
        if (addr < 0xFC00) {
            memory[addr] = value;
            return;
        }
        
        if (addr == 0xFFF9) { mapctl = value; return; }

        uint8_t page = addr >> 8;
        if (page == 0xFC) {
            if (!(mapctl & 0x01) && suzyWrite) { suzyWrite(suzyCtx, addr, value); return; }
            memory[addr] = value;
            return;
        }
        if (page == 0xFD) {
            if (!(mapctl & 0x02) && mikeyWrite) { mikeyWrite(mikeyCtx, addr, value); return; }
            memory[addr] = value;
            return;
        }
        
        // Oltre 0xFE00 le scritture vanno sempre in RAM
        memory[addr] = value;
    }


    // Registro MAPCTL (0xFFF9). Layout confermato da documentazione
    // hardware (monlynx.de, "App. 3: Hardware Addresses"). Reset = 0
    // (ROM/hardware visibili, come su hardware reale).
    uint8_t mapctl = 0x00;
    // Immagine della boot ROM (0xFE00-0xFFFF), READ-ONLY per la CPU.
    // Le scritture in questo range vanno SEMPRE a `memory[]` (la RAM
    // "nascosta" sotto la ROM, sempre presente sull'hardware reale),
    // mai a questo buffer.
    uint8_t bootRom[0x200] = {0};

    uint32_t sleepCyclesRemaining = 0;

};

#endif
