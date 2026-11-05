/*
 * lynx_perf.h - Piazzamento di codice e dati "caldi" nella SRAM interna
 * del RP2040, fuori dal percorso XIP (execute-in-place dalla flash).
 * 
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 *
 * PERCHE':
 * Sul RP2040 il codice risiede nella flash esterna e viene eseguito
 * attraverso la cache XIP da 16 KB (2 vie). L'emulatore ha tre working
 * set che da soli la superano: gli handler 65SC02 (~20 KB), il motore
 * sprite di Suzy (8 specializzazioni di drawLineT, ~15-25 KB) e la
 * sintesi audio di Mikey (~8 KB). Finche' stanno tutti in flash si
 * evitano a vicenda le linee di cache (thrashing): ogni passaggio
 * CPU->Suzy->CPU->audio ricomincia a mancare la cache, e ogni miss
 * XIP costa decine di cicli di stallo. Spostando in SRAM i tre blocchi
 * caldi, la flash resta solo per il codice freddo (setup, seriale,
 * blit) che da solo ci sta comodamente nella cache.
 *
 * COME:
 * Le sezioni .time_critical* sono raccolte dal linker script del
 * pico-sdk (memmap_default.ld, usato anche dal core Arduino-Pico)
 * dentro la sezione .data: RAM all'esecuzione, con immagine in flash
 * copiata all'avvio dal solito copy table. E' lo stesso meccanismo
 * documentato di __not_in_flash_func() del SDK. Si usano due nomi
 * distinti (codice/dati) perche' GCC rifiuta sezioni con lo stesso
 * nome ma flag diversi (conflitto AX vs A) nello stesso file.
 *
 * COSTO:
 * La SRAM consumata dal codice compare in Arduino come "Global
 * variables use". Se il link fallisse con "region RAM overflowed",
 * abbassa LYNX_RAM_LEVEL (sotto) e ricompila: e' il rubinetto di
 * sicurezza, non serve toccare i sorgenti.
 *
 * LIVELLI (modificabili anche da riga di comando: -DLYNX_RAM_LEVEL=n):
 *   0 = tutto in flash (baseline, per misurare il guadagno)
 *   1 = core CPU + Mikey audio in RAM, Suzy resta in flash
 *   2 = tutto il codice caldo in RAM (default)
 */

#ifndef LYNX_PERF_H
#define LYNX_PERF_H

#ifndef LYNX_RAM_LEVEL
#define LYNX_RAM_LEVEL 2
#endif

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350) || \
    defined(PICO_PLATFORM) || defined(PICO_BUILD)

  #if LYNX_RAM_LEVEL >= 1
    #define LYNX_RAMFUNC  __attribute__((section(".time_critical.piLynxCode")))
    #define LYNX_RAMDATA  __attribute__((section(".time_critical.piLynxData")))
  #else
    #define LYNX_RAMFUNC
    #define LYNX_RAMDATA
  #endif

  #if LYNX_RAM_LEVEL >= 2
    #define LYNX_RAMSPRITE __attribute__((section(".time_critical.piLynxCode")))
    // Sezione SEPARATA per le funzioni template (drawLineT<T>): le
    // istanze implicite sono simboli COMDAT/weak e GCC rifiuta
    // ("section type conflict") di mischiarle con le funzioni strong
    // nella stessa sezione. Il glob *(.time_critical*) del linker
    // script le raccoglie comunque insieme in RAM.
    #define LYNX_RAMSPRITE_TPL __attribute__((section(".time_critical.piLynxTpl")))
  #else
    #define LYNX_RAMSPRITE
    #define LYNX_RAMSPRITE_TPL
  #endif

#else
  /* Host/PC: macro vuote, il codice resta compilabile per test */
  #define LYNX_RAMFUNC
  #define LYNX_RAMDATA
  #define LYNX_RAMSPRITE
  #define LYNX_RAMSPRITE_TPL
#endif

#endif // LYNX_PERF_H
