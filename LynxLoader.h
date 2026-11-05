/*
 * Questo file fa parte del progetto PiLynx.
 *
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 *
 * LynxLoader - Caricamento ROM Lynx (.lnx) da SD (fallback LittleFS) con
 * caching "on demand" a blocchi da 4KB.
 *
 * PROTOCOLLO CARTUCCIA - porting da CCart di Handy (K. Wilkins). 
 * Punti chiave:
 * - UN SOLO shift register a 8 bit + UN SOLO contatore a 11 bit,
 *   condivisi fra i due banchi cartuccia.
 * - CART0 ($FCB2) e CART1 ($FCB3) sono banchi INDIPENDENTI (bank0/bank1
 *   del file .lnx)
 * - address_nel_banco = (shifter << shiftCount) + (counter & countMask)
 *   con shiftCount/countMask derivati dal campo header page_size_bankN.
 *
 * Il campo header page_size_bankN definisce la dimensione della "pagina"
 * hardware, quindi totalSize = page_size * 256 e' solo lo SPAZIO MASSIMO
 * indirizzabile dallo shifter a 8 bit, NON la quantita' di dati realmente
 * presenti nel file. Il campo actualSize contiene i byte EFFETTIVI
 * del banco nel file .lnx (split a meta' dei dati post-header, vedi
 * loadROM() nel .cpp). Le letture oltre actualSize ritornano 0xFF
 * (bus flottante, come leggere oltre la fine della ROM su hardware vero).
 */

#pragma once
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <LittleFS.h>

#define SD_CS    13
#define SD_MISO  12
#define SD_MOSI  11
#define SD_SCLK  10

// Dimensione del blocco di lettura/caching dalla SD (indipendente dalla
// "pagina" hardware della cartuccia, che e' un concetto diverso: qui e'
// solo granularita' del I/O bufferizzato).
static const uint32_t LYNX_IO_BLOCK_SIZE = 4096UL;

#ifndef LYNX_ROM_CACHE_EXTRA_BLOCKS
#define LYNX_ROM_CACHE_EXTRA_BLOCKS 4
#endif

enum LynxFSType { LYNX_FS_NONE, LYNX_FS_SD, LYNX_FS_LITTLEFS };
enum class LynxRotation : uint8_t { NONE = 0, LEFT = 1, RIGHT = 2 };

// Geometria di un banco cartuccia, derivata dall'header .lnx.
struct CartBankGeometry {
    bool     present   = false; // page_size_bankN != 0 nell'header
    uint32_t pageSize  = 0;     // = campo header (es. 0x100=256, 0x800=2048...)
    uint32_t shiftCount = 0;    // = log2(pageSize)
    uint32_t countMask  = 0;    // = pageSize - 1
    uint32_t totalSize  = 0;    // = spazio MASSIMO indirizzabile = pageSize * 256.
                                // SOLO maschera d'indirizzo hardware (& totalSize-1),
                                // NON la dimensione dei dati nel file!
    uint32_t actualSize = 0;    // byte REALI del banco nel file .lnx
    uint32_t fileOffset = 0;    // offset assoluto nel file dove iniziano i dati di questo banco
};

class LynxLoader {
public:
  LynxLoader();

  bool initFilesystem(LynxFSType preferred = LYNX_FS_SD);
  bool isReady() const { return _fsReady; }
  LynxFSType currentFS() const { return _currentFS; }

  void unloadROM();
  bool isLoaded() const { return _romLoaded; }

  // forcedPageSizeBank0: usato SOLO se il file non ha header .lnx valido
  // (magic "LYNX" assente). 0 = usa il default (1024, il formato piu'
  // comune per l'homebrew).
  // forcedRotation: idem, usato solo in assenza di header.
  bool loadROM(const char* path,
               uint16_t forcedPageSizeBank0 = 0,
               LynxRotation forcedRotation = LynxRotation::NONE);

  LynxRotation rotation() const { return _rotation; }

  uint32_t romSize()  const { return _romSize; }
  const char* romName() const { return _romName; }

  // --- EMULAZIONE HARDWARE CARTUCCIA ---
  // Scrittura bit-bang su SYSCTL1 (0xFD87, bit0=strobe) o IODAT
  // (0xFD8B, bit1=address data). Chiamata da Mikey::write().
  void writeCartReg(uint16_t mikeyOffset, uint8_t data);

  // Letture CPU-visibili via Suzy 0xFCB2 (bank0) / 0xFCB3 (bank1).
  // Auto-incrementano il contatore condiviso, come l'hardware reale.
  uint8_t readCart0Byte();
  uint8_t readCart1Byte();

  // Statistiche cache (debug)
  uint32_t cacheHits()  const { return _cacheHits; }
  uint32_t cacheMisses() const { return _cacheMisses; }

  void        setSelectedROM(const char* path);
  const char* getSelectedROM() const { return _romPath; }

  File openFile(const char* path, const char* mode);
  bool fileExists(const char* path);

private:
  bool initSD();
  bool openROMFile(const char* path);
  void closeROMFile();

  // Legge un byte a un offset ASSOLUTO nel file (gia' comprensivo di
  // eventuale header/offset di banco), gestendo cache/SD.
  uint8_t readFileByte(uint32_t fileOffset);

  bool cacheLookup(uint32_t blockIdx, uint8_t* dest);
  void cacheInsert(uint32_t blockIdx, const uint8_t* data);

  void estraiNome(const char* path);

  // Calcola shiftCount/countMask/totalSize a partire dal campo header
  // page_size_bankN (in byte). Ritorna false se pageSize non e' una
  // potenza di 2 valida (header corrotto).
  bool computeBankGeometry(uint16_t pageSizeField, CartBankGeometry& out);

  LynxRotation _rotation = LynxRotation::NONE;

  LynxFSType _currentFS = LYNX_FS_NONE;
  bool _fsReady = false;

  File _romFile;
  bool _romFileOpen = false;
  bool _romLoaded = false;

  uint32_t _romSize = 0;
  char _romName[64] = {0};
  char _romPath[128] = {0};

  // --- Stato hardware condiviso fra i due banchi (porting da CCart) ---
  uint32_t _shifter = 0;   // 8 bit
  uint32_t _counter = 0;   // 11 bit (mascherato 0x7FF)
  bool     _addrData = false;
  bool     _strobe   = false;

  CartBankGeometry _bank0;
  CartBankGeometry _bank1;

  // Buffer attivo e indice del blocco attualmente mappato in RAM
  uint8_t _blockBuf[LYNX_IO_BLOCK_SIZE];
  uint32_t _currentBlockIdx = 0xFFFFFFFFUL;

#if LYNX_ROM_CACHE_EXTRA_BLOCKS > 0
  uint8_t  _cachePool[LYNX_ROM_CACHE_EXTRA_BLOCKS][LYNX_IO_BLOCK_SIZE];
  uint32_t _cacheBlockIdx[LYNX_ROM_CACHE_EXTRA_BLOCKS];
  uint32_t _cacheLastUsed[LYNX_ROM_CACHE_EXTRA_BLOCKS];
  uint32_t _cacheClock = 0;
#endif

  uint32_t _cacheHits = 0;
  uint32_t _cacheMisses = 0;
};