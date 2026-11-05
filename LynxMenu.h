/*
 * Questo file fa parte del progetto PiLynx.
 *
 * Copyright (C) 2026 Ivan Vettori
 *
 * Porting di C64Menu (progetto Pi64) per l'emulatore Atari Lynx,
 * ridotto alle sole voci di gioco (niente BASIC, niente SID player):
 * il Lynx è una console, il menu serve solo a scegliere se aprire il
 * browser delle cartucce (.lnx) o riprendere la partita in corso.
 *
 * Rilasciato sotto licenza GNU General Public License v3.0.
 * Puoi ridistribuirlo e/o modificarlo secondo i termini della GPLv3.
 *
 * Questo programma è distribuito nella speranza che sia utile,
 * ma SENZA ALCUNA GARANZIA; senza neppure la garanzia implicita
 * di COMMERCIABILITÀ o IDONEITÀ PER UN PARTICOLARE SCOPO.
 * Vedi la licenza GPLv3 per maggiori dettagli.
 *
 * --------------------------------------------------------------------------
 *
 * Differenze rispetto a C64Menu (Pi64):
 * - voci menu: "GAMES BROWSER" + "RESUME GAME" (disabilitata/grigia
 *   finché non è stato avviato almeno un gioco: setGameLoaded());
 * - input: pad del Lynx via callback LynxPadReadFn invece di
 *   FunduinoJoystick (bit ATTIVI ALTI: vedi LynxPad.h). UP/DOWN
 *   navigano con wrap-around, INNER (A) conferma;
 *   la navigazione salta automaticamente le voci disabilitate;
 * - grafica: sfondo nero e accenti rosso/arancione (brand Lynx)
 *   al posto del blu/ciano del C64; aggiunta una riga hint in basso
 *   a sinistra con l'hotkey di uscita dal gioco (OPTION1+OPTION2).
 */

#ifndef LYNX_MENU_H
#define LYNX_MENU_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "LynxPad.h"

// ===== VOCI MENU (solo giochi) =====
enum LynxMenuItem {
    MENU_BROWSER = 0,   // "GAMES BROWSER": apre LynxGameBrowser
    MENU_RESUME  = 1,   // "RESUME GAME": torna alla partita in corso
    LYNX_MENU_ITEMS_COUNT = 2
};

// ===== AZIONI MENU =====
enum LynxMenuAction {
    ACTION_NONE = 0,
    ACTION_START_BROWSER = 1,
    ACTION_RESUME_GAME = 2
};

class LynxMenu {
public:
    LynxMenu(TFT_eSPI* display, LynxPadReadFn padReadFn, void* padCtx = nullptr);

    // Inizializza menu (disegna tutto)
    void begin();

    // Aggiorna menu (chiamare in loop).
    // Ritorna ACTION_NONE finché non viene confermata una voce.
    LynxMenuAction update();

    // Disegna menu completo
    void draw();

    // Imposta logo (opzionale, chiamare prima di begin())
    void setLogo(const unsigned short* logoData, int width, int height);

    // Abilita/disabilita la voce "RESUME GAME" (grigia se nessun gioco
    // è attivo). Se la voce selezionata viene disabilitata, la selezione
    // torna su "GAMES BROWSER".
    void setGameLoaded(bool loaded);

private:
    TFT_eSPI* tft;
    LynxPadReadFn padRead;
    void* padCtx;

    // Logo
    const unsigned short* logo;
    int logoWidth;
    int logoHeight;
    bool hasLogo;

    // Stato menu
    LynxMenuItem selectedItem;
    LynxMenuItem lastSelectedItem;
    bool resumeEnabled;

    // Debouncing pad
    uint32_t lastJoyTime;
    static const uint32_t JOY_DEBOUNCE_MS = 200;

    // Disegno
    void drawBackground();
    void drawLogo();
    void drawMenuItems();
    void drawCredits();
    void drawHint();
    void drawMenuItem(LynxMenuItem item, bool selected);

    // Calcolo posizioni
    int getMenuItemY(LynxMenuItem item);

    // Testi menu
    const char* getMenuItemText(LynxMenuItem item);

    // Navigazione
    bool isItemEnabled(LynxMenuItem item) const;
    LynxMenuItem stepItem(LynxMenuItem from, int dir) const;
};

#endif // LYNX_MENU_H
