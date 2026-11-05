/*
 * Questo file fa parte del progetto PiLynx.
 *
 * Copyright (C) 2026 Ivan Vettori
 *
 * Porting di C64Menu.cpp (progetto Pi64) per il solo menu giochi
 * dell'emulatore Atari Lynx. Vedi LynxMenu.h per le note sul porting.
 *
 * Rilasciato sotto licenza GNU General Public License v3.0.
 * Puoi ridistribuirlo e/o modificarlo secondo i termini della GPLv3.
 *
 * Questo programma è distribuito nella speranza che sia utile,
 * ma SENZA ALCUNA GARANZIA; senza neppure la garanzia implicita
 * di COMMERCIABILITÀ o IDONEITÀ PER UN PARTICOLARE SCOPO.
 * Vedi la licenza GPLv3 per maggiori dettagli.
 */

#include "LynxMenu.h"

LynxMenu::LynxMenu(TFT_eSPI* display, LynxPadReadFn padReadFn, void* ctx) {
    tft = display;
    padRead = padReadFn;
    padCtx = ctx;
    selectedItem = MENU_BROWSER;
    lastSelectedItem = MENU_BROWSER;
    resumeEnabled = false;
    lastJoyTime = 0;
    logo = nullptr;
    logoWidth = 0;
    logoHeight = 0;
    hasLogo = false;
}

void LynxMenu::setLogo(const unsigned short* logoData, int width, int height) {
    logo = logoData;
    logoWidth = width;
    logoHeight = height;
    hasLogo = (logo != nullptr);
}

void LynxMenu::setGameLoaded(bool loaded) {
    resumeEnabled = loaded;
    // Se la voce selezionata è appena stata disabilitata, sposta la
    // selezione sulla prima voce sempre attiva (GAMES BROWSER).
    if (!resumeEnabled && selectedItem == MENU_RESUME) {
        selectedItem = MENU_BROWSER;
        lastSelectedItem = MENU_BROWSER;
    }
}

void LynxMenu::begin() {
    Serial.println("LynxMenu: Init...");
    draw();
}

void LynxMenu::draw() {
    drawBackground();
    drawLogo();
    drawMenuItems();
    drawCredits();
    drawHint();
}

void LynxMenu::drawBackground() {
    // Il Lynx vive su sfondo nero (a differenza del blu del C64).
    tft->fillScreen(TFT_BLACK);
}

void LynxMenu::drawLogo() {
    if (!hasLogo) {
        // Se non c'è logo, mostra il testo in stile logo Lynx (rosso)
        tft->setTextColor(TFT_RED);
        tft->setTextSize(3);
        tft->setTextDatum(TC_DATUM); // Top Center
        tft->drawString("ATARI LYNX", 160, 20);
        return;
    }
    // centra logo orizzontalmente
    int logoX = (320 - logoWidth) / 2;
    int logoY = 20;
    // abilita swap bytes per RGB565
    tft->setSwapBytes(true);
    tft->pushImage(logoX, logoY, logoWidth, logoHeight, logo, 0x0000);
    tft->setSwapBytes(false);  // ripristina per altri usi
}

void LynxMenu::drawCredits() {
    tft->setTextSize(1);
    tft->setTextColor(TFT_CYAN);
    tft->setTextDatum(BR_DATUM); // basso a destra (BR)
    tft->drawString("@IV ATARI LYNX emulator", 315, 235);
}

void LynxMenu::drawHint() {
    // Aggiunta rispetto a C64Menu: sul C64 si usciva dal gioco con un
    // pulsante di reset fisico; qui l'hotkey va resa scopribile.
    tft->setTextSize(1);
    tft->setTextColor(TFT_DARKGREY);
    tft->setTextDatum(BL_DATUM); // basso a sinistra (BL)
    tft->drawString("OPT1+OPT2: back to menu", 5, 235);
}

void LynxMenu::drawMenuItems() {
    for (int i = 0; i < LYNX_MENU_ITEMS_COUNT; i++) {
        drawMenuItem((LynxMenuItem)i, i == selectedItem);
    }
}

void LynxMenu::drawMenuItem(LynxMenuItem item, bool selected) {
    int y = getMenuItemY(item);
    const char* text = getMenuItemText(item);
    bool enabled = isItemEnabled(item);

    // Cancella area voce (per aggiornamento)
    tft->fillRect(0, y - 2, 320, 26, TFT_BLACK);

    // Dimensione testo
    tft->setTextSize(2);
    tft->setTextDatum(TC_DATUM); // in alto centro (TC)

    if (selected && enabled) {
        // voce selezionata: disegna '>' prima del testo
        tft->setTextColor(TFT_WHITE);
        // calcola posizione del '>'
        int textWidth = strlen(text) * 12; // approssimazione larghezza
        int cursorX = (320 - textWidth) / 2 - 20; // 20px prima del testo
        tft->drawString(">", cursorX, y);
        tft->drawString(text, 160, y);
    } else {
        // voce non selezionata (o disabilitata: grigia scura)
        tft->setTextColor(enabled ? TFT_ORANGE : TFT_DARKGREY);
        tft->drawString(text, 160, y);
    }
}

int LynxMenu::getMenuItemY(LynxMenuItem item) {
    // Logo termina circa a Y=100 (20 + altezza logo)
    // Inizia menu da Y=110
    int startY = 110;
    int spacing = 35;
    return startY + (item * spacing);
}

const char* LynxMenu::getMenuItemText(LynxMenuItem item) {
    switch (item) {
        case MENU_BROWSER:
            return "GAMES BROWSER";
        case MENU_RESUME:
            return "RESUME GAME";
        default:
            return "???";
    }
}

bool LynxMenu::isItemEnabled(LynxMenuItem item) const {
    return (item == MENU_RESUME) ? resumeEnabled : true;
}

// Avanza di 'dir' passi (+1/-1) saltando le voci disabilitate,
// con wrap-around come su C64Menu.
LynxMenuItem LynxMenu::stepItem(LynxMenuItem from, int dir) const {
    LynxMenuItem it = from;
    for (int i = 0; i < LYNX_MENU_ITEMS_COUNT; i++) {
        it = (LynxMenuItem)((it + dir + LYNX_MENU_ITEMS_COUNT) % LYNX_MENU_ITEMS_COUNT);
        if (isItemEnabled(it)) return it;
    }
    return from; // nessuna voce abilitata oltre la corrente
}

LynxMenuAction LynxMenu::update() {
    if (!padRead) return ACTION_NONE;

    // Leggi pad (bit attivi alti, vedi LynxPad.h)
    uint8_t padState = padRead(padCtx);

    // debouncing: ignora input troppo veloci
    uint32_t now = millis();
    if (now - lastJoyTime < JOY_DEBOUNCE_MS) {
        return ACTION_NONE;
    }

    // UP: selezione precedente
    if (padState & LYNXPAD_UP) {
        lastJoyTime = now;
        selectedItem = stepItem(selectedItem, -1);
        // ridisegna solo se cambiato
        if (selectedItem != lastSelectedItem) {
            drawMenuItem(lastSelectedItem, false);
            drawMenuItem(selectedItem, true);
            lastSelectedItem = selectedItem;
        }
        return ACTION_NONE;
    }

    // DOWN: selezione successiva
    if (padState & LYNXPAD_DOWN) {
        lastJoyTime = now;
        selectedItem = stepItem(selectedItem, +1);
        // ridisegna solo se cambiato
        if (selectedItem != lastSelectedItem) {
            drawMenuItem(lastSelectedItem, false);
            drawMenuItem(selectedItem, true);
            lastSelectedItem = selectedItem;
        }
        return ACTION_NONE;
    }

    // INNER (A): conferma selezione
    if (padState & LYNXPAD_BTN_A) {
        lastJoyTime = now;
        if (!isItemEnabled(selectedItem)) {
            return ACTION_NONE; // voce disabilitata: nessuna azione
        }
        // Feedback visivo: lampeggio
        drawMenuItem(selectedItem, false);
        delay(100);
        drawMenuItem(selectedItem, true);
        delay(100);
        // Ritorna azione corrispondente
        switch (selectedItem) {
            case MENU_BROWSER:
                return ACTION_START_BROWSER;
            case MENU_RESUME:
                return ACTION_RESUME_GAME;
            default:
                return ACTION_NONE;
        }
    }
    return ACTION_NONE;
}
