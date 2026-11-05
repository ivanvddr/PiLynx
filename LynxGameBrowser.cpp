/*
 * Questo file fa parte del progetto PiLynx.
 *
 * Copyright (C) 2026 Ivan Vettori
 *
 * Porting di C64GameBrowser.cpp (progetto Pi64) per il solo browser
 * giochi dell'emulatore Atari Lynx. Vedi LynxGameBrowser.h per le note
 * sul porting, il layout della scheda SD e i fix applicati.
 *
 * Rilasciato sotto licenza GNU General Public License v3.0.
 * Puoi ridistribuirlo e/o modificarlo secondo i termini della GPLv3.
 *
 * Questo programma è distribuito nella speranza che sia utile,
 * ma SENZA ALCUNA GARANZIA; senza neppure la garanzia implicita di
 * COMMERCIABILITÀ O IDONEITÀ PER UN PARTICOLARE SCOPO.
 * Vedi la licenza GPLv3 per maggiori dettagli.
 */

#include "LynxGameBrowser.h"
#include <SDFS.h>   // oggetto fs::FS della SD (SDClass non espone openDir)

TFT_eSPI* LynxGameBrowser::staticTFT = nullptr;

char LynxGameBrowser::sortedFileNames[MAX_SORTED_GAMES][LNX_NAME_MAX];

LynxGameBrowser::LynxGameBrowser(TFT_eSPI* display, LynxLoader* loaderRef) {
    tft = display;
    loader = loaderRef;
    staticTFT = tft;
    currentFS = LYNX_FS_NONE;
    totalGames = 0;
    currentIndex = 0;
    currentFile[0] = '\0';
    lastJoyTime = 0;
    needsRedraw = true;
    backRequested = false;
    logo = nullptr;
    logoWidth = 0;
    logoHeight = 0;
    hasLogo = false;
    // FIX LANCIO ISTANTANEO: stato input di apertura
    aReleased = false;
    openTime = 0;
}

// ============================================================
// Inizializzazione
// ============================================================
bool LynxGameBrowser::begin(LynxFSType fsType) {
    Serial.println("LynxGameBrowser: Init...");
    if (browserPath[0] == '\0') {
        snprintf(browserPath, sizeof(browserPath), "%s%s", GAME_DIR, GAMES_SUBDIR);
    }
    if (browserExt[0] == '\0') {
        snprintf(browserExt, sizeof(browserExt), ".lnx");
    }
    // usa LynxLoader per init filesystem (idempotente: se il filesystem
    // è già pronto dall'avvio ritorna subito true, come su Pi64)
    if (!loader->initFilesystem(fsType)) {
        Serial.println("Filesystem unavailable!");
        showError("NO FILESYSTEM!");
        return false;
    }
    currentFS = loader->currentFS();
    Serial.printf("Filesystem: %s\n",
                  currentFS == LYNX_FS_SD ? "SD" : "LittleFS");

    // verifica directory
    Serial.printf("Path: %s\n", browserPath);
    Serial.flush();
    Serial.println("Check directory...");
    Serial.flush();
    File dirChk = openFile(browserPath, "r");
    if (!dirChk || !dirChk.isDirectory()) {
        Serial.println("Directory /PiLynx/games not found!");
        Serial.flush();
        showError("NO /PiLynx/games!");
        return false;
    }
    dirChk.close();
    Serial.println("Directory OK");
    Serial.flush();

    // conta giochi (scan veloce, solo nomi)
    totalGames = countLnxFiles();
    if (totalGames == 0) {
        Serial.println("no games found!");
        Serial.flush();
        showError("NO GAMES FOUND!");
        return false;
    }
    Serial.printf("found %d games\n", totalGames);
    Serial.flush();

    // costruisco array ordinato
    if (!buildSortedFileList()) {
        Serial.println("Unable to sort games list!");
        Serial.flush();
        showError("SORT ERROR!");
        return false;
    }

    // init decoder JPG
    TJpgDec.setJpgScale(1);
    TJpgDec.setCallback(tftOutput);

    // carica primo gioco
    currentIndex = 0;
    if (!getFileAtIndex(0, currentFile)) {
        Serial.println("Unable to load first game!");
        showError("LOAD ERROR!");
        return false;
    }
    Serial.printf("First Game: %s (index %d/%d)\n",
                  currentFile, currentIndex + 1, totalGames);

    // ------------------------------------------------------------
    // FIX LANCIO ISTANTANEO: reset dello stato di input.
    // 1) lastJoyTime parte da ADESSO: prima restava 0 e con millis()
    //    a ~37 s il controllo di debounce era aggirato al primo update().
    // 2) aReleased = false: update() resta "sordo" finché INNER (A) --
    //    il tasto con cui si è APERTO il browser dal menu -- non risulta
    //    rilasciato. Il click dura 200-300 ms: senza questa attesa veniva
    //    letto come "conferma" e il primo gioco partiva da solo.
    // ------------------------------------------------------------
    aReleased = false;
    openTime = millis();
    lastJoyTime = openTime;

    draw();
    return true;
}

// ============================================================
// popola array ordinato alfabeticamente
// ============================================================
bool LynxGameBrowser::buildSortedFileList() {
    if (totalGames == 0 || totalGames > MAX_SORTED_GAMES) {
        Serial.printf("Too many games for sorting (%d > %d)\n",
                      totalGames, MAX_SORTED_GAMES);
        return false;
    }
    Serial.printf("Building sorted array for %d games...\n", totalGames);
    Serial.flush();

    fs::FS* fs = getFS();
    if (!fs) {
        Serial.println("No filesystem for scan!");
        return false;
    }
    Dir dir = fs->openDir(browserPath);
    int count = 0;
    int entries = 0;
    uint32_t t0 = millis();

    while (dir.next() && count < totalGames) {
        entries++;
        if (dir.isFile()) {
            // Alcune implementazioni restituiscono il percorso completo:
            // usa solo il basename, come faceva la versione precedente.
            String fname = dir.fileName();
            const char* nm = fname.c_str();
            const char* base = strrchr(nm, '/');
            base = base ? base + 1 : nm;

            if (isLnx(base)) {
                if ((int)strlen(base) >= LNX_NAME_MAX) {
                    // Nome troppo lungo: verrebbe troncato e poi non piu'
                    // apribile. Meglio saltarlo con avviso.
                    Serial.printf("Skip (nome troppo lungo): %s\n", base);
                } else {
                    // copia diretta nello slot statico.
                    strncpy(sortedFileNames[count], base, LNX_NAME_MAX - 1);
                    sortedFileNames[count][LNX_NAME_MAX - 1] = '\0';
                    count++;
                }
            }
        }
        // Guardia anti-stallo: battito diagnostico ogni 16 voci e
        // timeout assoluto sulla scansione.
        if ((entries & 0x0F) == 0) {
            Serial.printf("  ... %d voci, %d .lnx (%lu ms)\n",
                          entries, count, (unsigned long)(millis() - t0));
            Serial.flush();
            if (millis() - t0 > BROWSER_SCAN_TIMEOUT_MS) {
                Serial.println("Lista: TIMEOUT, uso i nomi letti fin qui");
                Serial.flush();
                break;
            }
        }
    }
    // Ora un mismatch può avvenire solo se la directory cambia tra le
    // due letture o per il timeout: la memoria non c'entra più.
    if (count != totalGames) {
        Serial.printf("Count mismatch: expected %d, found %d\n", totalGames, count);
        totalGames = count;  // aggiusta
    }
    if (totalGames == 0) {
        return false;
    }

    // ordina alfabeticamente: insertion sort su array statico
    // (zero malloc, zero ricorsione, identico al risultato del bubble
    // sort di Pi64 che lavorava sui puntatori)
    Serial.println("Sorting...");
    char tmp[LNX_NAME_MAX];
    for (int i = 1; i < totalGames; i++) {
        strcpy(tmp, sortedFileNames[i]);
        int j = i - 1;
        while (j >= 0 && strcasecmp(sortedFileNames[j], tmp) > 0) {
            strcpy(sortedFileNames[j + 1], sortedFileNames[j]);
            j--;
        }
        strcpy(sortedFileNames[j + 1], tmp);
    }

    // debug: mostra primi 5
    Serial.println("Array sorted:");
    for (int i = 0; i < min(5, totalGames); i++) {
        Serial.printf("  [%d] %s\n", i, sortedFileNames[i]);
    }
    if (totalGames > 5) {
        Serial.println("  ...");
    }
    return true;
}

// CORE: accesso file per indice (usa array ordinato)
bool LynxGameBrowser::getFileAtIndex(int targetIndex, char* outName) {
    if (targetIndex < 0 || targetIndex >= totalGames) {
        Serial.printf("Index %d out of range (0-%d)\n", targetIndex, totalGames - 1);
        return false;
    }
    // ACCESSO DIRETTO all'array ordinato (O(1) invece di O(n))
    if (sortedFileNames[targetIndex][0] == '\0') {
        Serial.println("Sorted array not available!");
        return false;
    }
    strncpy(outName, sortedFileNames[targetIndex], LNX_NAME_MAX - 1);
    outName[LNX_NAME_MAX - 1] = '\0';
    Serial.printf("File[%d]: %s\n", targetIndex, outName);
    return true;
}

// ============================================================
// navigazione bidirezionale con Wrap-Around
// ============================================================
void LynxGameBrowser::nextGame() {
    if (totalGames == 0) return;
    int oldIndex = currentIndex;
    // forward con wrap
    currentIndex = (currentIndex + 1) % totalGames;
    Serial.printf("Navigation: %d -> %d (total: %d)\n",
                  oldIndex, currentIndex, totalGames);
    if (getFileAtIndex(currentIndex, currentFile)) {
        Serial.printf("New game: %s (%d/%d)\n",
                      currentFile, currentIndex + 1, totalGames);
        // navigazione istantanea (no scan, usa array!)
        needsRedraw = true;
        drawGameImage();
        drawGameName();
        drawCounter();
        needsRedraw = false;
    } else {
        // rollback se fallisce
        Serial.println("Loading failed, rollback");
        currentIndex = oldIndex;
    }
}

void LynxGameBrowser::prevGame() {
    if (totalGames == 0) return;
    int oldIndex = currentIndex;
    // backward con wrap (aggiunge totalGames per evitare negativi)
    currentIndex = (currentIndex - 1 + totalGames) % totalGames;
    Serial.printf("Navigation: %d -> %d (total: %d)\n",
                  oldIndex, currentIndex, totalGames);
    if (getFileAtIndex(currentIndex, currentFile)) {
        Serial.printf("New game: %s (%d/%d)\n",
                      currentFile, currentIndex + 1, totalGames);
        // Navigazione istantanea (no scan, usa array!)
        needsRedraw = true;
        drawGameImage();
        drawGameName();
        drawCounter();
        needsRedraw = false;
    } else {
        // Rollback se fallisce
        Serial.println("Loading failed, rollback");
        currentIndex = oldIndex;
    }
}

// update con pad (bit LYNXPAD_*, attivi alti)
bool LynxGameBrowser::update(uint8_t padState) {
    if (totalGames == 0) return false;
    uint32_t now = millis();

    // ------------------------------------------------------------
    // FIX LANCIO ISTANTANEO: il click (INNER/A) che ha aperto il
    // browser dal menu è ancora premuto nei primi 200-300 ms di vita
    // del browser: finché A non risulta RILASCIATO, TUTTO l'input
    // viene ignorato 
    // ------------------------------------------------------------
    if (!aReleased) {
        if (!(padState & LYNXPAD_BTN_A)) {
            // A rilasciato: da qui l'input viene accettato
            aReleased = true;
            lastJoyTime = now;
        } else if (now - openTime > BROWSER_RELEASE_TIMEOUT_MS) {
            // A inchiodato premuto (tasto/pin bloccato?): procedi
            // comunque, un avviso su Serial invece di un browser
            // che sembra congelato.
            Serial.println("WARNING: A ancora premuto dopo l'apertura (tasto bloccato?)");
            aReleased = true;
            lastJoyTime = now;
        }
        return false;
    }

    if (now - lastJoyTime < BROWSER_DEBOUNCE_MS) return false;
    if (padState & (LYNXPAD_LEFT | LYNXPAD_UP)) {   // gioco precedente
        lastJoyTime = now;
        prevGame();
        return false;
    }
    if (padState & (LYNXPAD_RIGHT | LYNXPAD_DOWN)) { // gioco successivo
        lastJoyTime = now;
        nextGame();
        return false;
    }
    if (padState & LYNXPAD_BTN_C) {                  // uscita al menu (OPTION2)
        lastJoyTime = now;
        backRequested = true;
        return false;
    }
    if (padState & LYNXPAD_BTN_A) {                  // conferma (INNER)
        lastJoyTime = now;
        return true;  // Gioco selezionato!
    }
    return false;
}

bool LynxGameBrowser::consumeBackRequest() {
    bool r = backRequested;
    backRequested = false;
    return r;
}

void LynxGameBrowser::setPath(const char* cpath) {
    snprintf(browserPath, sizeof(browserPath), "%s%s", GAME_DIR, cpath);
}

void LynxGameBrowser::setFileExt(const char* fext) {
    snprintf(browserExt, sizeof(browserExt), "%s", fext);
}

// Selezione gioco
String LynxGameBrowser::getLnxSelected() {
    char fullPath[96];
    snprintf(fullPath, sizeof(fullPath), "%s/%s", browserPath, currentFile);
    // Salva nel LynxLoader (come getPrgSelected faceva su Pi64)
    loader->setSelectedROM(fullPath);
    Serial.printf("File selected: %s\n", fullPath);
    return String(fullPath);
}

// ============================================================
// FILESYSTEM per l'enumerazione (FIX FREEZE SU LITTLEFS)
// ============================================================
// Ritorna l'oggetto fs::FS attivo: SDFS per la scheda SD (SDClass e' solo
// uno shim e non espone openDir), LittleFS per la flash interna. Con
// questo puntatore l'enumerazione usa l'API Dir.
fs::FS* LynxGameBrowser::getFS() {
    if (!loader) return nullptr;
    switch (loader->currentFS()) {
        case LYNX_FS_SD:
            return &SDFS;
        case LYNX_FS_LITTLEFS:
            return &LittleFS;
        default:
            return nullptr;
    }
}

int LynxGameBrowser::countLnxFiles() {
    fs::FS* fs = getFS();
    if (!fs) {
        Serial.println("Scan: filesystem non attivo!");
        Serial.flush();
        return 0;
    }
    Serial.println("Scan: apertura directory (API Dir)...");
    Serial.flush();

    Dir dir = fs->openDir(browserPath);
    int count = 0;
    int entries = 0;
    uint32_t t0 = millis();

    while (dir.next()) {
        entries++;
        if (dir.isFile() && isLnx(dir.fileName().c_str())) {
            count++;
            // Prime voci stampate sempre: se qualcosa va storto si vede
            // subito QUALI file sta leggendo.
            if (count <= 5) {
                Serial.printf("  gioco [%d] %s\n", count, dir.fileName().c_str());
                Serial.flush();
            }
        }
        // Guardia anti-stallo: battito ogni 16 voci + timeout assoluto.
        if ((entries & 0x0F) == 0) {
            Serial.printf("  ... %d voci, %d .lnx (%lu ms)\n",
                          entries, count, (unsigned long)(millis() - t0));
            Serial.flush();
            if (millis() - t0 > BROWSER_SCAN_TIMEOUT_MS) {
                Serial.println("Scan: TIMEOUT, interrompo (conteggio parziale)");
                Serial.flush();
                break;
            }
        }
    }

    Serial.printf("Scan completed: %d file %s (%d voci, %lu ms)\n",
                  count, browserExt, entries,
                  (unsigned long)(millis() - t0));
    Serial.flush();
    return count;
}

File LynxGameBrowser::openFile(const char* path, const char* mode) {
    return loader->openFile(path, mode);
}

bool LynxGameBrowser::fileExists(const char* path) {
    return loader->fileExists(path);
}

bool LynxGameBrowser::isLnx(const char* name) {
    int len = strlen(name);
    int extLen = (int)strlen(browserExt);
    if (extLen == 0 || len <= extLen) return false;
    return (strcasecmp(name + len - extLen, browserExt) == 0);
}

String LynxGameBrowser::getDisplayName(const char* filename) {
    String name = String(filename);
    int dotPos = name.lastIndexOf('.');
    return (dotPos > 0) ? name.substring(0, dotPos) : name;
}

bool LynxGameBrowser::hasImageFile(const char* lnxName) {
    String baseName = getDisplayName(lnxName);
    char jpgPath[96];
    snprintf(jpgPath, sizeof(jpgPath), "%s/%s.jpg", browserPath, baseName.c_str());
    return fileExists(jpgPath);
}

// ============================================================
// Disegno
// ============================================================
void LynxGameBrowser::draw() {
    if (!needsRedraw) return;
    drawBackground();
    drawLogo();
    drawGameImage();
    drawGameName();
    drawArrows();
    drawCounter();
    drawHint();
    drawCredits();
    needsRedraw = false;
}

void LynxGameBrowser::drawBackground() {
    // Sfondo nero
    tft->fillScreen(TFT_BLACK);
}

void LynxGameBrowser::drawLogo() {
    if (!hasLogo) {
        tft->setTextColor(TFT_ORANGE);
        tft->setTextSize(2);
        tft->setTextDatum(TL_DATUM);
        tft->drawString("PiLynx", 5, 5);
        return;
    }
    tft->setSwapBytes(true);
    tft->pushImage(5, 5, logoWidth, logoHeight, logo, 0x0000);
    tft->setSwapBytes(false);
}

void LynxGameBrowser::drawGameImage() {
    const int imgWidth = 200;
    const int imgHeight = 150;
    const int imgX = (320 - imgWidth) / 2;
    const int imgY = 40;
    tft->fillRect(imgX, imgY, imgWidth, imgHeight, TFT_BLACK);
    if (totalGames == 0 || currentFile[0] == '\0') return;
    String displayName = getDisplayName(currentFile);
    char imagePath[96];
    if (hasImageFile(currentFile)) {
        snprintf(imagePath, sizeof(imagePath), "%s/%s.jpg", browserPath, displayName.c_str());
    } else {
        snprintf(imagePath, sizeof(imagePath), "%s/noImage.jpg", browserPath);
    }
    if (!fileExists(imagePath)) {
        tft->drawRect(imgX, imgY, imgWidth, imgHeight, TFT_ORANGE);
        tft->setTextColor(TFT_ORANGE);
        tft->setTextSize(2);
        tft->setTextDatum(MC_DATUM);
        tft->drawString("NO IMAGE", 160, imgY + imgHeight / 2);
        return;
    }
    if (currentFS == LYNX_FS_SD) {
        TJpgDec.drawSdJpg(imgX, imgY, imagePath);
    } else if (currentFS == LYNX_FS_LITTLEFS) {
        TJpgDec.drawFsJpg(imgX, imgY, imagePath);
    }
}

void LynxGameBrowser::drawGameName() {
    if (totalGames == 0 || currentFile[0] == '\0') return;
    String displayName = getDisplayName(currentFile);
    tft->setTextColor(TFT_ORANGE);
    tft->setTextSize(2);
    tft->setTextDatum(TC_DATUM);
    tft->fillRect(0, 200, 320, 20, TFT_BLACK);
    // Nota: nomi più lunghi di ~26 caratteri vengono tagliati ai bordi
    // dello schermo da TFT_eSPI (stesso comportamento di Pi64).
    tft->drawString(displayName, 160, 200);
}

void LynxGameBrowser::drawArrows() {
    if (totalGames <= 1) return;
    tft->setTextColor(TFT_WHITE);
    tft->setTextSize(4);
    tft->setTextDatum(MC_DATUM);
    tft->drawString("<", 20, 120);
    tft->drawString(">", 300, 120);
}

// Aggiunto contatore posizione in alto adestra.
void LynxGameBrowser::drawCounter() {
    if (totalGames == 0 || currentFile[0] == '\0') return;
    char buf[16];
    snprintf(buf, sizeof(buf), "%d/%d", currentIndex + 1, totalGames);
    tft->setTextSize(2);
    tft->setTextColor(TFT_DARKGREY);
    tft->setTextDatum(TR_DATUM);
    tft->fillRect(228, 4, 92, 18, TFT_BLACK);
    tft->drawString(buf, 315, 5);
}

void LynxGameBrowser::drawCredits() {
    tft->setTextSize(1);
    tft->setTextColor(TFT_CYAN);
    tft->setTextDatum(BR_DATUM);
    tft->drawString("@IV ATARI LYNX emulator", 315, 235);
}

// Hint comandi in basso a sinistra (come nel menu): A avvia, C torna
// indietro senza cambiare gioco.
void LynxGameBrowser::drawHint() {
    tft->setTextSize(1);
    tft->setTextColor(TFT_DARKGREY);
    tft->setTextDatum(BL_DATUM);
    tft->drawString("A: play   C: back", 5, 235);
}

void LynxGameBrowser::showError(const char* msg) {
    tft->fillScreen(TFT_BLACK);
    tft->setTextColor(TFT_RED);
    tft->setTextSize(2);
    tft->setTextDatum(MC_DATUM);
    tft->drawString(msg, 160, 120);
    // A differenza di Pi64 (dove l'errore veniva subito sovrascritto dal
    // menu), qui si aspetta un attimo perché il messaggio sia leggibile.
    delay(1500);
}

void LynxGameBrowser::setLogo(const unsigned short* logoData, int width, int height) {
    logo = logoData;
    logoWidth = width;
    logoHeight = height;
    hasLogo = (logo != nullptr);
}

// Callback TJpg_Decoder: identica a Pi64 ma NON ripristina
// setSwapBytes(false): tutto PiLynx lavora con swap=true (il blit del
// framebuffer del loop1 lo richiede), lasciarlo attivo evita colori
// invertiti se il rendering riparte senza reinizializzarlo.
bool LynxGameBrowser::tftOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
    if (bitmap && staticTFT) {
        staticTFT->setSwapBytes(true);
        staticTFT->pushImage(x, y, w, h, bitmap);
    }
    return true;
}
