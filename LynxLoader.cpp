/*
 * Questo file fa parte del progetto PiLynx.
 *
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 *
 * Nel formato .lnx i dati che seguono l'header sono divisi in parti UGUALI 
 * fra i banchi presenti. Un solo banco = prende tutto; due banchi = meta' 
 * ciascuno. totalSize resta valido SOLO come maschera d'indirizzo hardware
 * (& totalSize-1); le letture oltre actualSize ritornano 0xFF.
 */

#include "LynxLoader.h"
#include <string.h>

// A 1 per loggare le prime letture RAW da CART0/CART1 (debug del
// protocollo cartuccia / verifica layout banchi). Default: disattivo.
#define LYNX_CART_DEBUG 0

LynxLoader::LynxLoader() {
#if LYNX_ROM_CACHE_EXTRA_BLOCKS > 0
  for (uint8_t i = 0; i < LYNX_ROM_CACHE_EXTRA_BLOCKS; i++) {
    _cacheBlockIdx[i] = 0xFFFFFFFFUL;
    _cacheLastUsed[i] = 0;
  }
#endif
}

// ============================================================================
// FILESYSTEM
// ============================================================================
bool LynxLoader::initFilesystem(LynxFSType preferred) {
  if (_fsReady) return true;

  Serial.println("LynxLoader: init filesystem...");

  auto trySD = [&]() -> bool {
    Serial.println("  tentativo SD...");
    if (initSD()) {
      _currentFS = LYNX_FS_SD; _fsReady = true;
      Serial.println("  SD OK"); return true;
    }
    Serial.println("  SD non disponibile"); return false;
  };
  auto tryLFS = [&]() -> bool {
    Serial.println("  tentativo LittleFS...");
    if (LittleFS.begin()) {
      _currentFS = LYNX_FS_LITTLEFS; _fsReady = true;
      Serial.println("  LittleFS OK"); return true;
    }
    Serial.println("  LittleFS non disponibile"); return false;
  };

  if (preferred == LYNX_FS_SD) {
    if (trySD()) return true;
    if (tryLFS()) return true;
  } else {
    if (tryLFS()) return true;
    if (trySD()) return true;
  }

  Serial.println("LynxLoader ERROR: nessun filesystem disponibile");
  return false;
}

bool LynxLoader::initSD() {
  SPI1.setRX(SD_MISO);
  SPI1.setTX(SD_MOSI);
  SPI1.setSCK(SD_SCLK);
  SPI1.setCS(SD_CS);
  SPI1.begin();
  delay(100);
  return SD.begin(SD_CS, 10000000, SPI1);
}

bool LynxLoader::openROMFile(const char* path) {
  if (!_fsReady) return false;
  if (_currentFS == LYNX_FS_SD) _romFile = SD.open(path, "r");
  else if (_currentFS == LYNX_FS_LITTLEFS) _romFile = LittleFS.open(path, "r");
  else return false;

  if (!_romFile) return false;
  _romFileOpen = true;
  return true;
}

void LynxLoader::closeROMFile() {
  if (_romFileOpen) {
    _romFile.close();
    _romFileOpen = false;
  }
}

// ============================================================================
// GEOMETRIA BANCO
// Il campo header page_size_bankN E' la dimensione (in byte) di ciascuna
// "pagina" indirizzata dal contatore a 11 bit; lo shifter e' sempre a 8
// bit, quindi lo SPAZIO MASSIMO indirizzabile del banco e' sempre
// pageSize*256. La dimensione REALE dei dati nel file e' calcolata a
// parte in loadROM() (actualSize).
// ============================================================================
bool LynxLoader::computeBankGeometry(uint16_t pageSizeField, CartBankGeometry& out) {
  out.present = false;
  out.pageSize = out.shiftCount = out.countMask = out.totalSize = out.actualSize = 0;

  if (pageSizeField == 0) return true; // banco assente, non e' un errore

  // pageSizeField deve essere una potenza di 2 (256, 512, 1024, 2048, ...)
  if ((pageSizeField & (pageSizeField - 1)) != 0) {
    Serial.printf("LynxLoader ERROR: page_size non e' potenza di 2 (0x%04X)\n", pageSizeField);
    return false;
  }

  uint32_t shift = 0;
  uint32_t tmp = pageSizeField;
  while (tmp > 1) { tmp >>= 1; shift++; }

  out.present    = true;
  out.pageSize   = pageSizeField;
  out.shiftCount = shift;
  out.countMask  = pageSizeField - 1;
  out.totalSize  = (uint32_t)pageSizeField << 8; // spazio indirizzabile: shifter a 8 bit -> *256
  return true;
}

// ============================================================================
// CARICAMENTO ROM
// ============================================================================
bool LynxLoader::loadROM(const char* path, uint16_t forcedPageSizeBank0, LynxRotation forcedRotation) {
  if (!path || path[0] == '\0') return false;
  if (!_fsReady && !initFilesystem()) return false;
  if (_romLoaded) unloadROM();

  Serial.printf("\nLynxLoader: carico > %s\n", path);

  if (!openROMFile(path)) return false;

  _romSize = _romFile.size();
  if (_romSize < 64 + 256) {
    Serial.println("LynxLoader ERROR: ROM troppo piccola / corrotta");
    closeROMFile();
    return false;
  }

  uint8_t header[64];
  if (_romFile.read(header, 64) != 64) {
    Serial.println("LynxLoader ERROR: impossibile leggere l'header");
    closeROMFile();
    return false;
  }

  bool hasHeader = (header[0] == 'L' && header[1] == 'Y' && header[2] == 'N' && header[3] == 'X');

  uint16_t pageSizeBank0, pageSizeBank1;
  uint32_t headerBytes;

  if (hasHeader) {
    pageSizeBank0 = (uint16_t)header[4] | ((uint16_t)header[5] << 8);
    pageSizeBank1 = (uint16_t)header[6] | ((uint16_t)header[7] << 8);
    uint16_t version = (uint16_t)header[8] | ((uint16_t)header[9] << 8);
    headerBytes = 64;

    // Rotazione: byte 0x3A (58) dell'header .lnx.
    uint8_t rot = header[0x3A];
    _rotation = (rot == 1) ? LynxRotation::LEFT
              : (rot == 2) ? LynxRotation::RIGHT
              : LynxRotation::NONE;

    if (version != 1) {
      Serial.printf("  ATTENZIONE: header .lnx version=%u (atteso 1), procedo comunque.\n", version);
    }
    Serial.printf("  Header .lnx OK: page_size_bank0=%u page_size_bank1=%u rotazione=%d\n",
                  pageSizeBank0, pageSizeBank1, (int)_rotation);
  } else {
    // Nessun header .lnx: dump "flat"/headerless. Uso l'override
    // esplicito se fornito, altrimenti il default piu' diffuso (1024
    // byte/blocco). Con un dump headerless la geometria 
    // e' indovinabile solo se noti la dimensione del file.
    pageSizeBank0 = (forcedPageSizeBank0 != 0) ? forcedPageSizeBank0 : 1024;
    pageSizeBank1 = 0;
    headerBytes = 0;
    _rotation = forcedRotation;
    Serial.printf("  Nessun header .lnx: fallback headerless, page_size_bank0=%u (forzato=%s), rotazione=%d\n",
                  pageSizeBank0, forcedPageSizeBank0 ? "si" : "no (default)", (int)_rotation);
  }

  if (!computeBankGeometry(pageSizeBank0, _bank0) ||
      !computeBankGeometry(pageSizeBank1, _bank1)) {
    closeROMFile();
    return false;
  }

  // ------------------------------------------------------------------
  // Layout dei banchi nel file:
  // i dati post-header sono divisi in parti UGUALI fra i banchi presenti.
  // ------------------------------------------------------------------
  uint32_t dataBytes = _romSize - headerBytes;
  uint32_t bank0Size = dataBytes; // default: un solo banco -> tutto a bank0

  if (_bank0.present && _bank1.present) {
    bank0Size = dataBytes / 2;
  }

  _bank0.fileOffset = headerBytes;
  _bank0.actualSize = _bank0.present ? bank0Size : 0;
  _bank1.fileOffset = headerBytes + (_bank0.present ? bank0Size : 0);
  _bank1.actualSize = _bank1.present ? (dataBytes - (_bank0.present ? bank0Size : 0)) : 0;

  // Sanity check: la geometria dichiarata deve riuscire a INDIRIZZARE
  // tutti i dati del banco (totalSize = page_size*256 >= actualSize).
  // Se non e' cosi', l'header e' quasi certamente sbagliato: avviso
  // invece di caricare dati che il gioco non potra' leggere.
  if (_bank0.present && _bank0.totalSize < _bank0.actualSize) {
    Serial.printf("  ATTENZIONE: bank0: spazio indirizzabile %lu B < dati reali %lu B: page_size header errato?\n",
                  (unsigned long)_bank0.totalSize, (unsigned long)_bank0.actualSize);
  }
  if (_bank1.present && _bank1.totalSize < _bank1.actualSize) {
    Serial.printf("  ATTENZIONE: bank1: spazio indirizzabile %lu B < dati reali %lu B: page_size header errato?\n",
                  (unsigned long)_bank1.totalSize, (unsigned long)_bank1.actualSize);
  }

  estraiNome(path);
  strncpy(_romPath, path, sizeof(_romPath) - 1);
  _romPath[sizeof(_romPath) - 1] = '\0';

  _shifter = 0;
  _counter = 0;
  _addrData = false;
  _strobe = false;

  _currentBlockIdx = 0xFFFFFFFFUL;
#if LYNX_ROM_CACHE_EXTRA_BLOCKS > 0
  for (uint8_t i = 0; i < LYNX_ROM_CACHE_EXTRA_BLOCKS; i++) {
    _cacheBlockIdx[i] = 0xFFFFFFFFUL;
    _cacheLastUsed[i] = 0;
  }
  _cacheClock = 0;
#endif
  _cacheHits = 0;
  _cacheMisses = 0;

  _romLoaded = true;
  Serial.printf("  Layout banchi: bank0 @file %lu (%lu B)%s | bank1 @file %lu (%lu B)\n",
                (unsigned long)_bank0.fileOffset, (unsigned long)_bank0.actualSize,
                _bank1.present ? "" : " (assente)",
                (unsigned long)_bank1.fileOffset, (unsigned long)_bank1.actualSize);
  Serial.printf("LynxLoader: ROM pronta > %s (bank0=%lu byte%s)\n",
                _romName, (unsigned long)(_bank0.present ? _bank0.totalSize : 0),
                _bank1.present ? ", bank1 presente" : "");
  return true;
}

void LynxLoader::unloadROM() {
  closeROMFile();
  _romLoaded = false;
  _romSize = 0;
  _romName[0] = '\0';
  _romPath[0] = '\0';
}

// ============================================================================
// HARDWARE CARTUCCIA 
// ============================================================================
void LynxLoader::writeCartReg(uint16_t mikeyOffset, uint8_t data) {
  if (mikeyOffset == 0x8B) { // IODAT: bit1 = Cart Address Data
    _addrData = (data & 0x02) != 0;
    return;
  }
  if (mikeyOffset == 0x87) { // SYSCTL1: bit0 = Cart Address Strobe
    bool strobe = (data & 0x01) != 0;

    // Fronte di salita: shifta un nuovo bit nello shift register a 8 bit.
    if (strobe && !_strobe) {
      _shifter = ((_shifter << 1) | (_addrData ? 1u : 0u)) & 0xFF;
    }
    // Mentre lo strobe e' asserito il contatore resta azzerato.
    if (strobe) {
      _counter = 0;
    }
    _strobe = strobe;
    return;
  }
}

uint8_t LynxLoader::readCart0Byte() {
  if (!_romLoaded || !_bank0.present) return 0xFF;
  uint32_t addrInBank = (_shifter << _bank0.shiftCount) + (_counter & _bank0.countMask);
  addrInBank &= (_bank0.totalSize - 1); // wrap nello spazio indirizzabile hardware

#if LYNX_CART_DEBUG
  {
    static uint32_t dbg = 0;
    if (dbg++ < 16)
      Serial.printf("CART0: shifter=%lu cnt=%lu fileOff=%lu (romSize=%lu)\n",
                    (unsigned long)_shifter, (unsigned long)_counter,
                    (unsigned long)(_bank0.fileOffset + addrInBank), (unsigned long)_romSize);
  }
#endif

  // Oltre i dati reali del banco c'e' solo bus flottante (0xFF).
  if (addrInBank >= _bank0.actualSize) return 0xFF;

  uint8_t b = readFileByte(_bank0.fileOffset + addrInBank);
  if (!_strobe) _counter = (_counter + 1) & 0x7FF;
  return b;
}

uint8_t LynxLoader::readCart1Byte() {
  if (!_romLoaded || !_bank1.present) return 0xFF;
  uint32_t addrInBank = (_shifter << _bank1.shiftCount) + (_counter & _bank1.countMask);
  addrInBank &= (_bank1.totalSize - 1); // wrap nello spazio indirizzabile hardware

#if LYNX_CART_DEBUG
  {
    static uint32_t dbg = 0;
    if (dbg++ < 16)
      Serial.printf("CART1: shifter=%lu cnt=%lu fileOff=%lu (romSize=%lu)%s\n",
                    (unsigned long)_shifter, (unsigned long)_counter,
                    (unsigned long)(_bank1.fileOffset + addrInBank), (unsigned long)_romSize,
                    (addrInBank >= _bank1.actualSize) ? "  <<<< OLTRE FINE DATI (0xFF)" : "");
  }
#endif

  // Otre i dati reali del banco c'e' solo bus flottante (0xFF).
  if (addrInBank >= _bank1.actualSize) return 0xFF;

  uint8_t b = readFileByte(_bank1.fileOffset + addrInBank);
  if (!_strobe) _counter = (_counter + 1) & 0x7FF;
  return b;
}

// ============================================================================
// I/O bufferizzato da SD/LittleFS 
// ============================================================================
uint8_t LynxLoader::readFileByte(uint32_t fileOffset) {
  if (!_romLoaded) return 0xFF;
  if (fileOffset >= _romSize) return 0xFF;

  uint32_t blockIdx = fileOffset / LYNX_IO_BLOCK_SIZE;
  uint32_t blockOff = fileOffset % LYNX_IO_BLOCK_SIZE;

  if (blockIdx == _currentBlockIdx) {
    return _blockBuf[blockOff];
  }

#if LYNX_ROM_CACHE_EXTRA_BLOCKS > 0
  if (cacheLookup(blockIdx, _blockBuf)) {
    _currentBlockIdx = blockIdx;
    _cacheHits++;
    return _blockBuf[blockOff];
  }
#endif
  _cacheMisses++;

  if (!_romFileOpen) return 0xFF;
  if (!_romFile.seek(blockIdx * LYNX_IO_BLOCK_SIZE)) return 0xFF;

  size_t n = _romFile.read(_blockBuf, LYNX_IO_BLOCK_SIZE);
  if (n == 0) return 0xFF;
  if (n < LYNX_IO_BLOCK_SIZE) {
    memset(_blockBuf + n, 0xFF, LYNX_IO_BLOCK_SIZE - n);
  }

  _currentBlockIdx = blockIdx;

#if LYNX_ROM_CACHE_EXTRA_BLOCKS > 0
  cacheInsert(blockIdx, _blockBuf);
#endif
  return _blockBuf[blockOff];
}

// ============================================================================
// CACHE LRU 
// ============================================================================
#if LYNX_ROM_CACHE_EXTRA_BLOCKS > 0
bool LynxLoader::cacheLookup(uint32_t blockIdx, uint8_t* dest) {
  for (uint8_t i = 0; i < LYNX_ROM_CACHE_EXTRA_BLOCKS; i++) {
    if (_cacheBlockIdx[i] == blockIdx) {
      memcpy(dest, _cachePool[i], LYNX_IO_BLOCK_SIZE);
      _cacheLastUsed[i] = ++_cacheClock;
      return true;
    }
  }
  return false;
}

void LynxLoader::cacheInsert(uint32_t blockIdx, const uint8_t* data) {
  for (uint8_t i = 0; i < LYNX_ROM_CACHE_EXTRA_BLOCKS; i++) {
    if (_cacheBlockIdx[i] == blockIdx) {
      memcpy(_cachePool[i], data, LYNX_IO_BLOCK_SIZE);
      _cacheLastUsed[i] = ++_cacheClock;
      return;
    }
  }
  uint8_t victim = 0;
  for (uint8_t i = 0; i < LYNX_ROM_CACHE_EXTRA_BLOCKS; i++) {
    if (_cacheBlockIdx[i] == 0xFFFFFFFFUL) { victim = i; break; }
    if (_cacheLastUsed[i] < _cacheLastUsed[victim]) victim = i;
  }
  memcpy(_cachePool[victim], data, LYNX_IO_BLOCK_SIZE);
  _cacheBlockIdx[victim] = blockIdx;
  _cacheLastUsed[victim] = ++_cacheClock;
}
#endif

// ============================================================================
// HELPER
// ============================================================================
void LynxLoader::estraiNome(const char* path) {
  if (!path) { _romName[0] = '\0'; return; }
  const char* slash = strrchr(path, '/');
  if (!slash) slash = strrchr(path, '\\');
  const char* base = slash ? slash + 1 : path;
  strncpy(_romName, base, sizeof(_romName) - 1);
  _romName[sizeof(_romName) - 1] = '\0';
  char* dot = strrchr(_romName, '.');
  if (dot) *dot = '\0';
}

File LynxLoader::openFile(const char* path, const char* mode) {
  if (!_fsReady) return File();
  if (_currentFS == LYNX_FS_SD)       return SD.open(path, mode);
  if (_currentFS == LYNX_FS_LITTLEFS) return LittleFS.open(path, mode);
  return File();
}

void LynxLoader::setSelectedROM(const char* path) {
  if (!path) return;
  strncpy(_romPath, path, sizeof(_romPath) - 1);
  _romPath[sizeof(_romPath) - 1] = '\0';
}

bool LynxLoader::fileExists(const char* path) {
  if (!_fsReady) return false;
  if (_currentFS == LYNX_FS_SD)       return SD.exists(path);
  if (_currentFS == LYNX_FS_LITTLEFS) return LittleFS.exists(path);
  return false;
}