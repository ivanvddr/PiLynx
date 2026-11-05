#ifndef BIOS_ROM_H
#define BIOS_ROM_H
#include <cstdint>

/*
 * Placeholder ROM LYNX
 *
 * SOSTITUIRE QUESTO FILE con la ROM reale.
 *
 *   Genera con:
 *     python3 rom2cpp.py lynxboot.img > bios_rom.h
 *
 *
 * Script rom2cpp.py:
 * ─────────────────────────────────────────────
 * import sys
 * data = open(sys.argv[1],'rb').read()
 * print('#ifndef BIOS_ROM_H')
 * print('#define BIOS_ROM_H')
 * print('#include <cstdint>')
 * print('static const uint8_t bios_rom[512] = {')
 * rows = [data[i:i+16] for i in range(0, len(data), 16)]
 * for row in rows:
 *     print('  ' + ', '.join(f'0x{b:02X}' for b in row) + ',')
 * print('};')
 * print(f'static const uint32_t bios_rom_len = sizeof(bios_rom);')
 * ─────────────────────────────────────────────
 *
 */

// Nel tuo progetto sostituisci con l'immagine vera (es. generata da lynxboot.img)
// La rom é protetta da copyright e non puó essere inclusa nell'emulatore
static const uint8_t bios_rom[512] = {    
  0x00
};
static const uint32_t bios_rom_len = sizeof(bios_rom);

#endif
