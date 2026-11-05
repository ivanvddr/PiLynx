/*
 * Questo file fa parte del progetto PiLynx.
 *
 * Copyright (C) 2026 Ivan Vettori
 *
 * Porting di C64GameBrowser (progetto Pi64) per il solo browser giochi
 * dell'emulatore Atari Lynx: cartucce .lnx sotto /PiLynx/games.
 *
 * Rilasciato sotto licenza GNU General Public License v3.0.
 * Puoi ridistribuirlo e/o modificarlo secondo i termini della GPLv3.
 *
 * Questo programma è distribuito nella speranza che sia utile,
 * ma SENZA ALCUNA GARANZIA; senza neppure la garanzia implicita di
 * COMMERCIABILITÀ O IDONEITÀ PER UN PARTICOLARE SCOPO.
 * Vedi la licenza GPLv3 per maggiori dettagli.
 *
 * --------------------------------------------------------------------------
 *
 * Layout scheda SD / LittleFS atteso:
 *
 *   /PiLynx/games/NomeGioco.lnx      cartuccia (obbligatoria)
 *   /PiLynx/games/NomeGioco.jpg      copertina 200x150 (opzionale)
 *   /PiLynx/games/noImage.jpg        copertina di fallback (opzionale)
 *
 */

#ifndef LYNXGAMEBROWSER_H
#define LYNXGAMEBROWSER_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include "LynxPad.h"
#include "LynxLoader.h"

#define GAME_DIR     "/PiLynx"
#define GAMES_SUBDIR "/games"
#define BROWSER_DEBOUNCE_MS 200
#define LNX_NAME_MAX  32      // Max caratteri del nome file (con NUL)

#define MAX_SORTED_GAMES 200

// Guardia anti-stallo per la scansione directory: se LittleFS/SD non
// risponde entro questo tempo la scansione si interrompe con avviso su
// Serial invece di congelare l'emulatore per sempre.
#define BROWSER_SCAN_TIMEOUT_MS 5000UL

// FIX LANCIO ISTANTANEO: attesa massima del rilascio di INNER (A) dopo
// l'apertura del browser. Scaduto il timeout il browser torna ad
// accettare l'input anche se A risulta ancora premuto (tasto/pin
// bloccato): meglio un avviso su Serial che un browser congelato.
#define BROWSER_RELEASE_TIMEOUT_MS 1500UL

class LynxGameBrowser {
public:
    LynxGameBrowser(TFT_eSPI* display, LynxLoader* loaderRef);
    // FIX HEAP: zero malloc => zero roba da liberare
    ~LynxGameBrowser() {}

    bool begin(LynxFSType fsType = LYNX_FS_SD);
    void setLogo(const unsigned short* logoData, int width, int height);
    void draw();
    void nextGame();
    void prevGame();

    // Aggiorna browser con lo stato del pad (bit LYNXPAD_*).
    // Ritorna true quando un gioco è stato confermato (INNER/A).
    bool update(uint8_t padState);

    // True (una volta sola) se è stato richiesto l'uscita dal browser
    // con OPTION2 (C). 
    bool consumeBackRequest();

    // Contatori per diagnostica (non usati dallo sketch)
    int getTotalGames() const { return totalGames; }
    int getCurrentIndex() const { return currentIndex; }

    // Percorso completo del .lnx selezionato; lo salva anche nel
    // LynxLoader (setSelectedROM) come faceva getPrgSelected su Pi64.
    String getLnxSelected();

    void setPath(const char* cpath);    // relativo a GAME_DIR, es. "/games"
    void setFileExt(const char* fext);  // es. ".lnx"

private:
    TFT_eSPI* tft;
    LynxLoader* loader;
    LynxFSType currentFS;

    char browserPath[32] = {0};
    char browserExt[8]   = {0};

    const unsigned short* logo;
    int logoWidth;
    int logoHeight;
    bool hasLogo;

    // array ordinato alfabeticamente
    int totalGames;
    int currentIndex;
    char currentFile[LNX_NAME_MAX];
    uint32_t lastJoyTime;
    bool needsRedraw;
    bool backRequested;

    static char sortedFileNames[MAX_SORTED_GAMES][LNX_NAME_MAX];

    // FIX LANCIO ISTANTANEO: false finché INNER (A) non risulta rilasciato
    // dopo l'apertura del browser (il click sul menu non deve confermare).
    bool aReleased;
    uint32_t openTime;

    // filesystem (delega al LynxLoader dell'emulatore)
    File openFile(const char* path, const char* mode = "r");
    bool fileExists(const char* path);

    // Filesystem "grezzo" per l'enumerazione con l'API Dir
    // (LittleFS su flash, SDFS su scheda SD). Null se non pronto.
    fs::FS* getFS();

    // sorting e accesso ordinato
    int countLnxFiles();
    bool buildSortedFileList();
    bool getFileAtIndex(int index, char* outName);

    // utility
    bool isLnx(const char* name);
    String getDisplayName(const char* filename);
    bool hasImageFile(const char* lnxName);

    // disegno
    void drawBackground();
    void drawLogo();
    void drawGameImage();
    void drawGameName();
    void drawArrows();
    void drawCounter();
    void drawHint();
    void drawCredits();
    void showError(const char* msg);

    // callback TJpg_Decoder
    static bool tftOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap);
    static TFT_eSPI* staticTFT;
};

#endif // LYNXGAMEBROWSER_H
