/*
 * Questo file fa parte del progetto PiLynx.
 *
 * Copyright (C) 2026 Ivan Vettori
 *
 * Opera originale derivata dal progetto Pi64 (C64Menu/C64GameBrowser),
 * adattata al solo browser giochi dell'emulatore Atari Lynx.
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
 * LynxPad - astrazione minima del pad condivisa da LynxMenu e
 * LynxGameBrowser: un byte di stato con bit ATTIVI ALTI (bit=1 -> tasto
 * premuto), costruito dal main (.ino) a partire dai propri pin già
 * debounced (il struct DebouncedPin di PiLynx.ino). Sul C64 il ruolo era
 * di FunduinoJoystick::read(); qui non esiste una classe joystick, quindi
 * il main fornisce direttamente il byte mappato sui bit LYNXPAD_*.
 *
 * Il debounce di contatto (15 ms) resta a carico del caller; il
 * "rateo di ripetizione" della navigazione (200 ms) resta a carico di
 * LynxMenu / LynxGameBrowser, esattamente come su Pi64.
 */

#ifndef LYNX_PAD_H
#define LYNX_PAD_H

#define LYNXPAD_UP    0x01  // su
#define LYNXPAD_DOWN  0x02  // giù
#define LYNXPAD_LEFT  0x04  // sinistra
#define LYNXPAD_RIGHT 0x08  // destra
#define LYNXPAD_BTN_A 0x10  // INNER   (pulsante A) : conferma
#define LYNXPAD_BTN_B 0x20  // OPTION1 (pulsante B) : hotkey uscita
#define LYNXPAD_BTN_C 0x40  // OPTION2 (pulsante C) : hotkey uscita

// Callback di lettura del pad. Firma identica a quella usata da Suzy per
// gli hook (ctx + valore di ritorno), così il main registra una funzione
// sola. LynxMenu la invoca internamente da update(); il browser riceve
// invece lo stato direttamente in update(uint8_t), come su Pi64.
typedef uint8_t (*LynxPadReadFn)(void* ctx);

#endif // LYNX_PAD_H
