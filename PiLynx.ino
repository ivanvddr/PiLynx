/*
 * PiLynx.ino — Emulatore Lynx per Raspberry Pi Pico (RP2040), Arduino IDE
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 *
 * Architettura:
 * - CORE0 (setup/loop): CPU 65SC02 + Mikey + Suzy + input + rendering.
 *   Produce i campioni audio (audioProducer.step) nel ring SPSC.
 * - CORE1 (setup1/loop1): pump audio (ring -> libreria I2S) + upscale x2
 *   + invio SPI. L'uscita audio e' la libreria I2S del core (PIO + DMA)
 *   verso il MAX98357A: il clock audio e' il bit-clock I2S a 22050 Hz
 *   ESATTI (PIO, divider frazionario).
 *
 */

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <I2S.h>
#include <new>   // placement-new per il "power cycle" cpu/mikey/suzy

#include "cpu65sc02.h"
#include "mikey.h"
#include "LynxLoader.h"
#include "suzy.h"
#include "ring_buffer_spsc.h"
#include "audio_pipeline.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"

// Menu + browser giochi
#include "PiLynxLogo.h"
#include "LynxPad.h"
#include "LynxMenu.h"
#include "LynxGameBrowser.h"

#pragma GCC optimize("O3")

// ============================================================================
// PIN AUDIO (uscita I2S verso MAX98357A)
// ============================================================================
#define AUDIO_BCLK_PIN 26   // BCLK. La libreria genera LRCLK/WS sul GPIO
                            // adiacente (27): non e' impostabile separatamente.
#define AUDIO_DATA_PIN 28   // DIN verso il DAC.
// Collegamenti: BCLK->26, LRC->27, DIN->28, VIN 5V, GND comune.
// Uscita BTL: l'altoparlante SOLO tra + e -, mai verso GND.
// Con L==R (mono duplicato) e' irrilevante lo strap del pin SD
// dell'ampli (left / right / L+R mix).

// NOTA (rev. realtime): FRAMESKIP e' ora il MINIMO garantito: se
// l'emulazione non tiene il passo, lo skip ADATTIVO (vedi loop())
// alza da solo il periodo di render. Priorita' dichiarata: timing reale > framerate.
#define FRAMESKIP 1
int frameCounter = 0;

// ==================== PACING / GUARDRAIL (rev. realtime) ====================
// Debito massimo di tempo emulato recuperabile senza balzi (us): oltre,
// la deadline si riaggancia al presente. Valore basso = recupero breve
// dopo un picco (es. sprite lungo), al prezzo di un piccolo scatto.
#define PACING_MAX_DEBT_US       30000UL

// Massimo anticipo della deadline sul tempo reale (us). Evita che un frame
// molto lungo (es. boot ROM) lasci la deadline lontana nel futuro.
#define PACING_MAX_AHEAD_US 20000UL
bool firstFrame=true;
uint32_t deadline = 0;

// SCORTA MINIMA del ring audio, in campioni (128 = ~5,8 ms @ 22050 Hz):
// l'attesa attiva del pacing si interrompe SEMPRE se il contenuto del
// ring scende sotto questo livello. E' il cuore del guardrail: il
// throttle non puo' MAI mandare l'audio a secco, perche' l'attesa si
// trasforma immediatamente in produzione. Copre anche la finestra
// cieca del pump per-strip di core1 (~2,5 ms).
#define AUDIO_GUARDRAIL_MIN_LEAD 128

// Ritardo oltre il quale il frame NON viene renderizzato: il tempo di
// CPU va all'emulazione, non al disegno (timing > framerate).
#define PACING_RENDER_SKIP_US    8000

// Massimo numero di frame "dovuti" consecutivi che il guardrail di
// ritardo (o il blit ancora in corso) puo' saltare. Raggiunto il limite
// il render viene FORZATO anche se sono in ritardo (se core1 sta ancora
// facendo il blit, parte appena si libera). 0 = nessun limite (sconsigliato:
// con un ritardo cronico lo schermo si congela).
// Distanza massima tra due render ~ effSkip * (PACING_MAX_CONSECUTIVE_SKIP + 1)
// frame Lynx.
#define PACING_MAX_CONSECUTIVE_SKIP 3

// Campioni massimi pompati verso l'I2S dopo ogni pushImageDMA di strip
// (loop1). Ogni campione in piu' = fino a ~45us di stallo se la lib e'
// piena. 48 campioni ~ 2,2 ms: circa la durata del trasferimento SPI di
// una strip. Valori alti allungano il blit (e abbassano gli fps video):
// 128 campioni x 12 strip arrivavano a ~70 ms per frame.
#define AUDIO_STRIP_PUMP_CAP 48

#include "bios_rom.h"   // const uint8_t bios_rom[] e bios_rom_len
                        // (immagine della boot ROM Lynx, 512 byte)

#define ROM_LOAD_ADDR 0x0000

#define DISPLAY_W 320
#define DISPLAY_H 240

#define PICO_CLOCK_KHZ 300000

volatile bool core1CanStart = false;

// Diagnostica clock: frequenza RICHIESTA vs frequenza REALE
static uint32_t g_clkSysKhz = 0;
static bool     g_clkSetOk  = false;

// Frame "dovuti" consecutivi non disegnati (globale solo per il log).
static uint8_t g_skipRun = 0;

// ============================================================================
// INPUT
// ============================================================================
#define PIN_UP     2
#define PIN_DOWN   3
#define PIN_LEFT   4
#define PIN_RIGHT  5
#define PIN_CENTER 22  // 5a via del direzionale -> OUTER
#define PIN_BTN_A  6   // -> JOY_INNER
#define PIN_BTN_B  8   // -> JOY_OPTION1
#define PIN_BTN_C  0   // -> JOY_OPTION2
#define PIN_PAUSE  7   // -> SWITCHES

// ============================================================================
// DEBUG / PROFILING
// ============================================================================
#define DEBUG_PROFILING 0

#if DEBUG_PROFILING
  static uint32_t mikeyTime = 0, audioTime = 0;
  static uint32_t sliceCount = 0;
  static uint32_t lastPrint = 0;
  #define DBG_TIME_START(var) uint32_t _t_##var = micros()
  #define DBG_TIME_END(var)   var += (micros() - _t_##var)
#else
  #define DBG_TIME_START(var)
  #define DBG_TIME_END(var)
#endif

// SOLO in profiling (4 KB di RAM e una scrittura in
// più per slice: in release non devono costare nulla).
#if DEBUG_PROFILING
static uint32_t pcHist[1024];
#endif

struct DebouncedPin {
    uint8_t pin;
    bool stableState = false;
    bool lastRead = false;
    uint32_t lastChangeMs = 0;
};

static DebouncedPin inputPins[9] = {
    {PIN_UP}, {PIN_DOWN}, {PIN_LEFT}, {PIN_RIGHT}, {PIN_CENTER},
    {PIN_BTN_A}, {PIN_BTN_B}, {PIN_BTN_C}, {PIN_PAUSE}
};

#define DEBOUNCE_MS 15

// Istanza della libreria I2S del core Arduino-Pico.
I2S i2sOut(OUTPUT);

TFT_eSPI tft = TFT_eSPI();

// ============================================================================
// VARIABILI GLOBALI PER IL DOPPIO CORE
// ============================================================================
#define LYNX_W 160
#define LYNX_H 102
#define FRAME_BUF_SIZE (LYNX_W * LYNX_H)

// REV. RAM (buffer unico): frameBufB eliminato (-32.640 B). Sicuro perche'
// il render e' vietato finche' frameReady e' attivo: render e blit non si
// sovrappongono mai, quindi il framebuffer non viene mai scritto mentre
// core1 lo legge.
uint16_t frameBufA[FRAME_BUF_SIZE];

volatile uint16_t* renderFramePtr = frameBufA;   // buffer unico: sempre frameBufA
volatile uint16_t* displayFramePtr = nullptr;
volatile bool frameReady = false;
volatile int framesSkipped = 0;

#define STRIP_H 20
static uint16_t stripBuf[DISPLAY_W * STRIP_H];

uint32_t totalFrameTime = 0;
uint32_t cpuTime = 0;
uint32_t renderTime = 0;

// ============================================================================
// EMULATORE
// ============================================================================
cpu65sc02 cpu;
Mikey mikey;
Suzy suzy;

#define SLICE_CYCLES 400

void decryptLynxBlock(uint8_t* data, uint32_t size) {
    uint16_t lfsr = 0x0089;
    for (uint32_t i = 0; i < size; i++) {
        uint8_t byte = 0;
        for (int bit = 0; bit < 8; bit++) {
            uint8_t carry = lfsr & 1;
            lfsr >>= 1;
            if (carry) lfsr ^= 0x2C89;
            byte = (byte << 1) | (lfsr & 1);
        }
        data[i] ^= byte;
    }
}

// ============================================================================
// AUDIO (I2S -> MAX98357A)
//
// Produttore: core0 (audioProducer.step nel loop).
// Consumatore: core1, qui. Due punti di pompaggio:
//   1) per-strip, tra pushImageDMA e dmaWait: MENTRE lo SPI viaggia;
//   2) nel ramo else di loop1, quando non c'e' blit in corso.
// write16() blocca quando i buffer della libreria sono pieni: e' il
// freno/backpressure
// ============================================================================
AudioRing audioRing;
AudioProducer audioProducer;
AudioConsumer audioConsumer;

LynxLoader loader;

void cartBankCallback(void* ctx, uint16_t offset, uint8_t value) {
    LynxLoader* l = static_cast<LynxLoader*>(ctx);
    l->writeCartReg(offset, value);
}

#define AUDIO_SAMPLE_RATE 22050

// --- Switch diagnostici (entrambi a 0 per il funzionamento normale) ---
// Contatori: prod scritto da core0, pump da core1. Word 32bit allineate:
// letture/scritture atomiche su Cortex-M0+, quindi stamparli/azzerarli
// dal profiling su core0 e' sicuro (al piu' si perde un conteggio).
volatile uint32_t audioProducedCount = 0;  // campioni generati
volatile uint32_t audioPumpedCount   = 0;  // campioni consegnati alla lib

// Mono: stesso campione su L e R. write16 blocca SOLO a buffer pieno
// (~45us di attesa per campione): il consumo a 22050 esatti fa' da
// orologio al sistema
void writeI2sSample(int16_t sample, void* /*ctx*/) {
    audioPumpedCount++;
    i2sOut.write16(sample, sample);
}

// Pump ring -> libreria I2S. Non e' un busy-loop: esce appena il ring e'
// vuoto. Il CAP limita i campioni per chiamata: se la lib e' piena,
// write16 blocca ~45us per campione, quindi un backlog grosso non puo'
// sequestrare a lungo il blit video (il cap per strip e'
// AUDIO_STRIP_PUMP_CAP, e lo stallo cade DENTRO il trasferimento SPI
// della strip, che intanto procede in hardware).
void pumpAudio(uint32_t maxSamples) {
    while (maxSamples-- && !audioRing.isEmpty()) {
        audioConsumer.tick(audioRing);
    }
}

// chiamata da Suzy durante un disegno sprite lungo. Converte il
// tempo reale trascorso in cicli CPU equivalenti (4MHz -> 4 cicli/us) e fa
// avanzare SOLO la sintesi audio (non i timer), cosi' il canale che stava
// suonando continua a farlo durante lo stallo invece di congelarsi.
// waitIfFull=false: se il ring si riempie, scarta il piu' vecchio invece
// di bloccare (sono dentro il rendering sprite, non voglio rallentarlo
// ulteriormente aspettando che core1 consumi).
void suzyAudioCatchup(void* /*ctx*/, uint32_t elapsedUs) {
    uint32_t cycles = elapsedUs * 4;
    audioProducedCount += (uint32_t)audioProducer.step((int)cycles, audioRing, false);
}

void setupInputPins() {
    for (auto& p : inputPins) {
        pinMode(p.pin, INPUT_PULLUP);
    }
}

void updateDebounce(DebouncedPin& p, uint32_t nowMs) {
    bool raw = !digitalRead(p.pin);
    if (raw != p.lastRead) {
        p.lastRead = raw;
        p.lastChangeMs = nowMs;
    } else if (raw != p.stableState && (nowMs - p.lastChangeMs) >= DEBOUNCE_MS) {
        p.stableState = raw;
    }
}

void pollInput() {
    uint32_t nowMs = millis();
    for (auto& p : inputPins) updateDebounce(p, nowMs);

    uint8_t joy = 0;
    if (inputPins[0].stableState) joy |= JOY_UP;
    if (inputPins[1].stableState) joy |= JOY_DOWN;
    if (inputPins[2].stableState) joy |= JOY_LEFT;
    if (inputPins[3].stableState) joy |= JOY_RIGHT;
    if (inputPins[4].stableState) joy |= JOY_OUTER;
    if (inputPins[5].stableState) joy |= JOY_INNER;
    if (inputPins[6].stableState) joy |= JOY_OPTION1;
    if (inputPins[7].stableState) joy |= JOY_OPTION2;
    suzy.setJoystick(joy);

    uint8_t sw = inputPins[8].stableState ? SWITCH_PAUSE : 0;
    suzy.setSwitches(sw);
}

// Non piu' usata dal flusso menu->browser (le ROM arrivano dalla SD via
// LynxLoader), mantenuta per test/debug con bios_rom.h incorporato.
void loadRom() {
    uint32_t len = bios_rom_len;
    if (ROM_LOAD_ADDR + len > 0x10000) {
        len = 0x10000 - ROM_LOAD_ADDR;
    }
    memcpy(cpu.memory + ROM_LOAD_ADDR, bios_rom, len);
}

// ============================================================================
// MENU PRINCIPALE + BROWSER GIOCHI
//
// Macchina a stati identica nella struttura a quella del Pi64:
//   STATE_MENU      -> LynxMenu ("GAMES BROWSER" / "RESUME GAME")
//   STATE_BROWSER   -> LynxGameBrowser (/PiLynx/games/*.lnx)
//   STATE_EMULATION -> il vecchio corpo di loop(), intatto
// Dal gioco si torna al menu con OPTION1+OPTION2 (B+C) tenuti insieme
// per EXIT_COMBO_MS; lo stato del gioco NON viene toccato, quindi
// "RESUME GAME" riprende esattamente da dove si era lasciato.
// ============================================================================
enum AppState {
    STATE_MENU,
    STATE_BROWSER,
    STATE_EMULATION
};

AppState currentState = STATE_MENU;

LynxMenu*        menu        = nullptr;
LynxGameBrowser* gameBrowser = nullptr;

// True dopo il primo lancio: abilita "RESUME GAME" nel menu.
bool gameLoaded = false;

// Handshake bus SPI del TFT fra core0 (menu/browser) e core1 (blit).
// core1 setta core1BlitActive PRIMA di controllare core1BlitPaused:
// cosi', quando core0 vede core1BlitActive==false con la pausa gia'
// alzata, e' garantito che core1 non tocchera' piu' lo SPI finche'
// la pausa non viene tolta. L'audio (write16/I2S) non passa dallo SPI
// del TFT e continua sempre.
volatile bool core1BlitPaused = false;  // core0 -> core1
volatile bool core1BlitActive = false;  // core1 -> core0

// Durata della pressione congiunta OPTION1+OPTION2 per uscire dal gioco.
#define EXIT_COMBO_MS 800

// Sospende l'uso dello SPI da parte di core1 e attende (max 300 ms, il
// blit completo dura ~15-31 ms) che l'eventuale strip in volo termini.
void pauseCore1Blit() {
    core1BlitPaused = true;
    uint32_t t0 = millis();
    while (core1BlitActive && (millis() - t0 < 300)) {
        tight_loop_contents();
    }
}

void resumeCore1Blit() {
    core1BlitPaused = false;
}

// Pulizia del display al cambio di "schermata chiude la transazione SPI, aspetta eventuali DMA in volo e
// riporta il display in uno stato pulito per il disegno del menu.
void cleanupTFT() {
    Serial.println("Cleanup TFT...");
    tft.endWrite();
    uint32_t timeout = millis();
    while (tft.dmaBusy() && (millis() - timeout < 200)) {
        delay(1);
    }
    tft.fillScreen(TFT_BLACK);
    delay(50); // aspetta che fillScreen finisca
    tft.setTextDatum(TL_DATUM);
    Serial.println("TFT clean");
}

// Callback di lettura pad per LynxMenu (e stato per il browser):
// riusa il debounce esistente dei pin e lo mappa sui bit LYNXPAD_*.
uint8_t menuPadRead(void* /*ctx*/) {
    uint32_t nowMs = millis();
    for (auto& p : inputPins) updateDebounce(p, nowMs);

    uint8_t pad = 0;
    if (inputPins[0].stableState) pad |= LYNXPAD_UP;
    if (inputPins[1].stableState) pad |= LYNXPAD_DOWN;
    if (inputPins[2].stableState) pad |= LYNXPAD_LEFT;
    if (inputPins[3].stableState) pad |= LYNXPAD_RIGHT;
    if (inputPins[5].stableState) pad |= LYNXPAD_BTN_A;   // INNER
    if (inputPins[6].stableState) pad |= LYNXPAD_BTN_B;   // OPTION1
    if (inputPins[7].stableState) pad |= LYNXPAD_BTN_C;   // OPTION2
    return pad;
}

// Esce dal gioco e torna al menu SENZA toccare lo stato emulato
// (cpu/mikey/suzy restano congelati: "RESUME GAME" riprende da li').
void goToMenu() {
    Serial.println("Uscita dal gioco -> menu");
    cpu.Halt(false);          // igiene: mai rientrare in emulazione halted
    pauseCore1Blit();         // core1 via dallo SPI del TFT
    frameReady = false;       // nessun blit pendente
    framesSkipped = 0;

    cleanupTFT();

    if (!menu) {
        menu = new LynxMenu(&tft, menuPadRead, nullptr);
    }
    menu->setLogo(logoPiLynx,143,70);
    menu->setGameLoaded(gameLoaded);
    menu->draw();

    // core1 puo' riprendere: con frameReady==false fara' solo pump audio
    resumeCore1Blit();
    currentState = STATE_MENU;
}

// Riprende il gioco congelato (nessun reset: si torna esattamente allo
// stato in cui si era usciti con il combo B+C).
void resumeEmulation() {
    Serial.println("RESUME: riprendo il gioco");
    cpu.Halt(false);
    // core1 e' gia' in pausa-only-audio (o ci va ora): pulisco lo
    // schermo dal menu PRIMA di ridargli il bus SPI.
    pauseCore1Blit();
    tft.setSwapBytes(true);   // il blit del framebuffer lo richiede
    tft.fillScreen(TFT_BLACK);
    tft.startWrite();
    firstFrame = true;        // il pacing si riaggancia al presente
    g_skipRun = 0;
    resumeCore1Blit();
    currentState = STATE_EMULATION;
}

// "Power cycle" + cambio cartuccia: l'equivalente di spegnere la
// console, sostituire la .lnx e riaccendere. Richiamata dal browser
// dopo la conferma di un gioco.
void launchGame(const char* fullPath) {
    Serial.printf("\nLynxLauncher: avvio > %s\n", fullPath);

    pauseCore1Blit();
    frameReady = false;
    cleanupTFT();

    // ----------------------------------------------------------------
    // STEP 1: reset hardware emulato ("power cycle").
    // I placement-new ricostruiscono cpu/mikey/suzy nello stato di
    // power-on (registri, timer, IRQ pending, canali audio, MAPCTL=0,
    // penIndex...). Gli oggetti sono GLOBALI: il loro indirizzo non
    // cambia, quindi gli hook registrati dentro la CPU
    // (attachSuzy/attachMikey) e il puntatore passato ad
    // audioProducer.begin() restano validi; li re-registro comunque
    // per esplicita'. memory[] e' azzerata a mano (il costruttore non
    // la tocca), la boot ROM viene ricaricata.
    // ----------------------------------------------------------------
    loader.unloadROM();              // chiude il file della ROM precedente
    new (&cpu)   cpu65sc02();
    new (&mikey) Mikey();
    new (&suzy)  Suzy();
    memset(cpu.memory, 0, RAMSIZE_LYNX);
    cpu.loadBootRom(bios_rom, bios_rom_len);   // boot ROM invariata

    mikey.attachTo(cpu);
    mikey.attachSuzy(&suzy);
    suzy.attachTo(cpu);
    suzy.setAudioCatchup(suzyAudioCatchup, nullptr);
    mikey.attachCart(&loader, cartBankCallback);
    suzy.attachCart(&loader);

    // ----------------------------------------------------------------
    // STEP 2: carica la cartuccia selezionata dal browser.
    // ----------------------------------------------------------------
    if (!loader.loadROM(fullPath)) {
        Serial.println("LynxLauncher ERROR: loadROM fallita");
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_RED);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("LOAD ERROR!", 160, 120);
        delay(2000);
        gameLoaded = false;
        goToMenu();
        return;
    }

    mikey.setRotation(loader.rotation());
    suzy.setRotation(loader.rotation());

    // ----------------------------------------------------------------
    // STEP 3: reset CPU: il vettore di reset ($FFFC) arriva dalla boot
    // ROM (MAPCTL=0 dopo il power cycle), che legge l'header della
    // cartuccia via CART0 e avvia il gioco.
    // ----------------------------------------------------------------
    cpu.resetCPU();

    // Reset dei contatori di pacing/diagnostica
    firstFrame = true;
    frameCounter = 0;
    framesSkipped = 0;
    g_skipRun = 0;
    gameLoaded = true;

    // ----------------------------------------------------------------
    // STEP 4: display pronto per il blit video di core1, via libera.
    // ----------------------------------------------------------------
    tft.setSwapBytes(true);
    tft.fillScreen(TFT_BLACK);
    tft.startWrite();
    resumeCore1Blit();
    currentState = STATE_EMULATION;
    Serial.printf("LynxLauncher: gioco avviato (%s)\n", loader.romName());
}

// ============================================================================
// STATO: MENU PRINCIPALE
// ============================================================================
void handleMenuState() {
    if (!menu) {
        // Sicurezza: non dovrebbe mai accadere (il menu nasce in setup)
        menu = new LynxMenu(&tft, menuPadRead, nullptr);
        menu->setLogo(logoPiLynx,143,70);
        menu->setGameLoaded(gameLoaded);
        menu->begin();
    }

    LynxMenuAction action = menu->update();
    switch (action) {
        case ACTION_START_BROWSER:
            Serial.println("Menu: apertura browser giochi...");
            currentState = STATE_BROWSER;
            break;
        case ACTION_RESUME_GAME:
            if (gameLoaded) {
                resumeEmulation();
            }
            break;
        case ACTION_NONE:
        default:
            // Nessuna azione
            break;
    }
    delay(10);
    yield();
}

// ============================================================================
// STATO: BROWSER GIOCHI
// ============================================================================
void handleBrowserState() {
    // Crea browser solo una volta (come su Pi64)
    if (!gameBrowser) {
        gameBrowser = new LynxGameBrowser(&tft, &loader);
        gameBrowser->setLogo(logoPiLynx_small,60,30);
        gameBrowser->setPath(GAMES_SUBDIR);    // -> /PiLynx/games
        gameBrowser->setFileExt(".lnx");
        if (!gameBrowser->begin()) {
            Serial.println("Browser init failed!");
            delete gameBrowser;
            gameBrowser = nullptr;
            currentState = STATE_MENU;
            if (menu) menu->draw();
            return;
        }
        Serial.println("Browser giochi attivo");
    }

    // Aggiorna browser con lo stato del pad
    bool gameSelected = gameBrowser->update(menuPadRead(nullptr));
    if (gameSelected) {
        Serial.println("Game selected!");
        // Il browser salva il path nel Loader e ce lo
        // restituisce; poi libera l'heap della lista ordinata.
        String selectedPath = gameBrowser->getLnxSelected();
        delete gameBrowser;
        gameBrowser = nullptr;
        launchGame(selectedPath.c_str());
        return;
    }
    // OPTION2 (C) nel browser = torna al menu senza cambiare gioco
    if (gameBrowser->consumeBackRequest()) {
        Serial.println("Browser: ritorno al menu");
        delete gameBrowser;
        gameBrowser = nullptr;
        currentState = STATE_MENU;
        if (menu) menu->draw();
        return;
    }
    delay(10);
    yield();
}

// ============================================================================
// STATO: EMULAZIONE
// (il corpo del vecchio loop(), intatto, piu' il combo di uscita B+C)
// ============================================================================
void handleEmulationState() {
    uint32_t frameStart = micros();
    uint32_t cyclesThisFrame = 0;   // REV: cicli emulati del frame (pacing)
#if DEBUG_PROFILING
    cpuTime = 0; mikeyTime = 0; audioTime = 0; renderTime = 0; sliceCount = 0;
#endif

    // 1. Esegui CPU finche' il VBL non scatta (frame logico del Lynx)
    while (!mikey.consumeVblFlag()) {
        DBG_TIME_START(cpuTime);
        int cyclesRan = cpu.execCPU(SLICE_CYCLES);
        DBG_TIME_END(cpuTime);
        cyclesThisFrame += (uint32_t)cyclesRan;
#if DEBUG_PROFILING
        sliceCount++;
#endif

        DBG_TIME_START(mikeyTime);
        mikey.step(cyclesRan);
        #if DEBUG_PROFILING
        pcHist[cpu.getpc() >> 6]++;
        #endif
        DBG_TIME_END(mikeyTime);

        DBG_TIME_START(audioTime);
        // step() ritorna i campioni prodotti: li conto per la
        // diagnostica (prod/s atteso ~ AUDIO_SAMPLE_RATE).
        audioProducedCount += (uint32_t)audioProducer.step(cyclesRan, audioRing);
        DBG_TIME_END(audioTime);
    }
    pollInput();

    // ================================================================
    // 1b. USCITA DAL GIOCO: OPTION1 + OPTION2 (pulsanti B+C) tenuti
    //     insieme per EXIT_COMBO_MS. I due tasti restano passati al
    //     gioco (che li vede come OPTION normali): il combo e' solo un
    //     hotkey, non altera lo stato emulato.
    // ================================================================
    {
        static uint32_t comboStartMs = 0;
        bool combo = inputPins[6].stableState && inputPins[7].stableState;
        if (combo) {
            if (comboStartMs == 0) {
                comboStartMs = millis();
            } else if (millis() - comboStartMs >= EXIT_COMBO_MS) {
                comboStartMs = 0;
                goToMenu();
                return;
            }
        } else {
            comboStartMs = 0;
        }
    }

    // comandi diagnostici da seriale.
    // 1..4 = silenzia/riattiva canale audio N; 0 = riattiva tutti; d = dump stato audio.
    while (Serial.available()) {
        int c = Serial.read();
        if (c >= '1' && c <= '4') mikey.audioMuteMask ^= (1 << (c - '1'));
        else if (c == '0') mikey.audioMuteMask = 0;
        else if (c == 'd') mikey.dumpAudio();
    }

    // ================================================================
    // 2. PACING: deadline cumulativa al tempo emulato + MISURA DEL RITARDO, ora calcolata PRIMA di
    //    decidere se renderizzare. emulatedUs = cicli/4 (4 MHz).
    // ================================================================
    int32_t latenessUs = 0;
    uint32_t frameDeadline = 0;
    {
        uint32_t now = micros();
        if (firstFrame) { deadline = frameStart; firstFrame = false; }   // era: now
        if ((int32_t)(now - deadline) > (int32_t)PACING_MAX_DEBT_US) {
            deadline = now - PACING_MAX_DEBT_US;
        }
        deadline += (cyclesThisFrame >> 2);
        // anticipo massimo: la deadline non puo' stare piu' di AHEAD_US avanti
        if ((int32_t)(deadline - micros()) > (int32_t)PACING_MAX_AHEAD_US) {
            deadline = micros() + PACING_MAX_AHEAD_US;
        }
        frameDeadline = deadline;
        latenessUs = (int32_t)(micros() - deadline);
    }

    // ================================================================
    // 3. RENDERING adattativo e consapevole del ritardo.
    //    a) se sono in forte ritardo (>= PACING_RENDER_SKIP_US) il
    //       frame non si disegna: il tempo di CPU serve all'emulazione;
    //       MA dopo PACING_MAX_CONSECUTIVE_SKIP frame dovuti saltati
    //       di fila il render viene forzato (il display non si congela);
    //    b) se la velocita' media resta sotto il 95%, lo skip adattivo
    //       alza il periodo di render (misura il rapporto lavoro/ tempo
    //       emulato ogni 500 ms, con isteresi sui gradini 1/2/3/4);
    //    c) FRAMESKIP resta il MINIMO: mai peggiorare il refresh se
    //       c'e' margine.
    // ================================================================
    {
        static uint32_t accEmulUs = 0, accWorkUs = 0;
        static uint32_t lastAdaptMs = 0;
        static uint8_t dynamicSkip = 1;

        accEmulUs += (cyclesThisFrame >> 2);
        accWorkUs += (micros() - frameStart);   // lavoro puro, senza pacing
        if (millis() - lastAdaptMs >= 500) {
            lastAdaptMs = millis();
            uint32_t pct = accWorkUs ? (accEmulUs * 100UL) / accWorkUs : 100;
            dynamicSkip = (pct >= 95) ? 1 : (pct >= 85) ? 2 : (pct >= 70) ? 3 : 4;
            accEmulUs = 0;
            accWorkUs = 0;
        }

        uint8_t effSkip = (FRAMESKIP > dynamicSkip) ? FRAMESKIP : dynamicSkip;
        frameCounter++;
        if (frameCounter >= effSkip) {
            const bool late   = latenessUs >= (int32_t)PACING_RENDER_SKIP_US;
            const bool forced = (PACING_MAX_CONSECUTIVE_SKIP > 0) &&
                                (g_skipRun >= PACING_MAX_CONSECUTIVE_SKIP);

            // frameReady = core1 sta ancora facendo il blit del precedente:
            // in quel caso non si puo' renderizzare, ma g_skipRun continua
            // a contare e il render parte appena core1 si libera.
            if (!frameReady && (!late || forced)) {
                frameCounter = 0;
                g_skipRun = 0;
                DBG_TIME_START(renderTime);
                // Buffer unico: nessuna swap. Il render arriva qui solo
                // quando !frameReady (core1 ha gia' finito il blit
                // precedente), quindi scrivere frameBufA e' sicuro; core1
                // lo legge solo finche' frameReady resta attivo.
                mikey.renderFrameRGB565((uint16_t*)renderFramePtr);
                displayFramePtr = renderFramePtr;
                frameReady = true;
                DBG_TIME_END(renderTime);
            } else {
                if (g_skipRun < 255) g_skipRun++;
                framesSkipped++;
            }
        }
    }

    uint32_t totalFrameTime = micros() - frameStart; // solo lavoro, senza pacing

    // ================================================================
    // 4. THROTTLE + GUARDRAIL AUDIO: attesa attiva fino alla deadline,
    //    MA solo finche' il ring audio conserva la scorta minima.
    //
    //    lead = campioni prodotti - campioni consegnati a I2S: e' il
    //    contenuto reale del ring (l'orologio audio e' il bit-clock
    //    I2S del PIO, non micros()). Se lead scende sotto la scorta,
    //    l'attesa si interrompe e il frame successivo produce campioni:
    //    - quando l'emulazione corre PIU' VELOCE del tempo reale, si
    //      aggancia al ritmo dell'audio reale, senza mai svuotarlo;
    //    - quando e' in ritardo (deadline passata) non si aspetta
    //      nulla: massima velocita' di recupero;
    //    - qualunque deriva sistematica tra micros() e il clock I2S
    //      viene assorbita qui (self-correcting), non si accumula.
    // ================================================================
    while ((int32_t)(micros() - frameDeadline) < 0) {
        int32_t lead = (int32_t)(audioProducedCount - audioPumpedCount);
        if (lead < (int32_t)AUDIO_GUARDRAIL_MIN_LEAD) break;
        tight_loop_contents();
    }

#if DEBUG_PROFILING
    {
        static uint32_t secEmulUs = 0, secWallUs = 0;
        secEmulUs += (cyclesThisFrame >> 2);
        secWallUs += (micros() - frameStart);   // include il pacing
        if (millis() - lastPrint >= 1000) {
            lastPrint = millis();
            uint32_t effMhz10 = (cpuTime > 0) ? (cyclesThisFrame * 10UL) / cpuTime : 0;
            uint32_t speedPct = secWallUs ? (secEmulUs * 100UL) / secWallUs : 0;

            Serial.println("--- PROFILING REALE ---");
            Serial.printf("Frame totale: %u us (emulato: %u us)\n", totalFrameTime, cyclesThisFrame >> 2);
            Serial.printf("  Velocita': %u%% | CPU eff: %u.%u MHz\n",
                          speedPct, effMhz10 / 10, effMhz10 % 10);
            Serial.printf("  Ritardo: %ld us | skipRun: %u (max %d) | frameReady: %d\n",
                          (long)latenessUs, (unsigned)g_skipRun,
                          (int)PACING_MAX_CONSECUTIVE_SKIP, (int)frameReady);
            Serial.printf("  CLK: set_sys_clock_khz=%d, clk_sys REALE=%lu kHz\n",
                          (int)g_clkSetOk, (unsigned long)g_clkSysKhz);
            Serial.printf("  CPU:       %u us\n", cpuTime);
            Serial.printf("  Mikey:     %u us\n", mikeyTime);
            Serial.printf("  Audio:     %u us\n", audioTime);
            // prod = campioni generati da core0, pump = consegnati alla
            // libreria da core1. In regime (velocita' ~100%): entrambi
            // ~22050. prod < 22050 -> l'emulazione non tiene il ritmo
            // (buchi inevitabili). prod ~ 22050 ma audio rotto con
            // TONE+NOBLIT -> colpa della catena I2S/libreria.
            // prod/pump come DIFFERENZE su contatori monotonici (mai
            // azzerati: il guardrail del pacing li legge a ogni frame).
            static uint32_t lastProd = 0, lastPump = 0;
            uint32_t prod = audioProducedCount - lastProd; lastProd = audioProducedCount;
            uint32_t pump = audioPumpedCount - lastPump;   lastPump = audioPumpedCount;
            Serial.printf("  Audio: prod=%lu pump=%lu (atteso ~%u)\n",
                          (unsigned long)prod,
                          (unsigned long)pump, AUDIO_SAMPLE_RATE);
            Serial.printf("  Audio underrun (consumer): %lu\n",
                          (unsigned long)audioConsumer.getUnderrunCount());
            audioConsumer.resetUnderrunCount();
            Serial.printf("  Render:    %u us\n", renderTime);
            Serial.printf("  Slice:     %u\n", sliceCount);
            Serial.printf("  Skip:      %d\n", framesSkipped);
            Serial.printf("  Underflow: %lu\n", (unsigned long)mikey.getUnderflowCount());
            Serial.printf("  PC=0x%04X status=0x%02X\n", cpu.getpc(), cpu.getStatus());
            Serial.printf("  IRQline: %lu\n", (unsigned long)mikey.irqLineCount);
            Serial.printf("  Uflow/T: %lu %lu %lu %lu %lu %lu %lu %lu\n",
                (unsigned long)mikey.underflowPerTimer[0], (unsigned long)mikey.underflowPerTimer[1],
                (unsigned long)mikey.underflowPerTimer[2], (unsigned long)mikey.underflowPerTimer[3],
                (unsigned long)mikey.underflowPerTimer[4], (unsigned long)mikey.underflowPerTimer[5],
                (unsigned long)mikey.underflowPerTimer[6], (unsigned long)mikey.underflowPerTimer[7]);
            mikey.resetUnderflowPerTimer();
            mikey.irqLineCount = 0;
            mikey.resetUnderflowCount();

            for (int n = 0; n < 3; n++) {
                int best = -1;
                for (int i = 0; i < 1024; i++)
                    if (best < 0 || pcHist[i] > pcHist[best]) best = i;
                if (best < 0 || pcHist[best] == 0) break;
                Serial.printf("  HOT PC %04X-%04X: %lu slice\n",
                              best << 6, (best << 6) + 0x3F, (unsigned long)pcHist[best]);
                pcHist[best] = 0;
            }
            memset(pcHist, 0, sizeof(pcHist));

            mikey.traceDump();
            suzy.traceDump();
            Serial.printf("  SERDAT writes: %lu\n", (unsigned long)mikey.serdatWrites);
            mikey.serdatWrites = 0;
            framesSkipped = 0;
            secEmulUs = 0;
            secWallUs = 0;
        }
    }
#endif
}

// ============================================================================
// CORE0: setup
// ============================================================================
void setup() {

    vreg_set_voltage(VREG_VOLTAGE_1_30);
    // REV: catturo il risultato. Se ritorna false (o clk_sys non
    // corrisponde poi a PICO_CLOCK_KHZ) il sistema e' rimasto al clock
    // di default e tutte le metriche vanno lette con QUEL clock.
    g_clkSetOk = set_sys_clock_khz(PICO_CLOCK_KHZ, true);
    uint32_t f = clock_get_hz(clk_sys);
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, f, f);
    __sync_synchronize();

    delay(100);

    // La frequenza REALE, non quella richiesta: e' il numero che conta.
    g_clkSysKhz = clock_get_hz(clk_sys) / 1000UL;

    // Input: serve al menu prima ancora che all'emulatore
    setupInputPins();

    // Display: il menu lo usa subito
    tft.init();
    tft.setRotation(1);
    tft.initDMA();
    tft.fillScreen(TFT_BLACK);
    tft.setSwapBytes(true);

    // Boot ROM Lynx (512 byte): indipendente dalla cartuccia, resta
    // valida per tutte le sessioni (launchGame la ricarica a ogni
    // "power cycle" insieme al reset dell'hardware).
    cpu.loadBootRom(bios_rom, bios_rom_len);

    // Filesystem (SD con fallback LittleFS): lo usa il browser. Un
    // fallimento qui NON e' fatale, verra' segnalato dal browser
    // all'apertura con "NO FILESYSTEM!" (initFilesystem e' idempotente).
    loader.initFilesystem();

    // Produttore audio pronto ma inattivo: nel menu non girano cicli
    // emulati, il ring resta vuoto e l'I2S emette silenzio.
    AudioConfig cfg;
    cfg.sampleRate = AUDIO_SAMPLE_RATE;
    audioProducer.begin(cfg, &mikey);
    #if AUDIO_TEST_TONE
        audioProducer.setDiagnosticOverride(generateSample, nullptr);
    #endif

    // Menu principale: stato iniziale della macchina a stati.
    // Nessuna ROM viene caricata qui: si parte sempre dal menu.
    menu = new LynxMenu(&tft, menuPadRead, nullptr);
    menu->setLogo(logoPiLynx,143,70);
    menu->setGameLoaded(false);
    menu->begin();

    // Core1 via libera: I2S + pump audio (silenzio finche' un gioco
    // non parte; nessun blit video perche' frameReady==false)
    core1CanStart = true;

    Serial.println("Setup completato: menu pronto");
}

// ============================================================================
// CORE0: loop
// ============================================================================
void loop() {
    switch (currentState) {
        case STATE_MENU:
            handleMenuState();
            break;
        case STATE_BROWSER:
            handleBrowserState();
            break;
        case STATE_EMULATION:
            handleEmulationState();
            break;
    }
}

// ============================================================================
// CORE1: pump audio + upscale + invio SPI
// ============================================================================
void setup1() {
    while (!core1CanStart) { tight_loop_contents(); }

    i2sOut.setBCLK(AUDIO_BCLK_PIN);   // LRCLK/WS = 26+1 = 27, generato
    i2sOut.setDATA(AUDIO_DATA_PIN);   // automaticamente dalla libreria
    i2sOut.setBitsPerSample(16);
    // Capacita' dei buffer interni: 4 x 256 parole. ATTENZIONE: capacita'
    // != scorta. La scorta reale e' l'anticipo intra-frame del produttore
    // (~0-6ms): il pacing non lascia correre core0 avanti. Questa
    // profondita' serve solo ad assorbire la finestra cieca residua del
    // pump per-strip (~1 strip, ~2,5ms). Con il fix si potrebbe anche
    // ridurla (es. 4 x 64) per latenza piu' bassa.
    i2sOut.setBuffers(4, 256, 0);
    // PIO + divider frazionario: 22050 ESATTI.
    i2sOut.begin(AUDIO_SAMPLE_RATE);

    audioConsumer.begin(writeI2sSample, nullptr);
}

void loop1() {
    // HANDSHAKE SPI con core0 (menu/browser): dichiara il possibile uso
    // dello SPI PRIMA di controllare la pausa. core0, che aspetta
    // core1BlitActive==false con la pausa gia' alzata, ha cosi' la
    // garanzia che core1 non iniziera' un blit mentre menu/browser
    // stanno disegnando (vedi pauseCore1Blit).
    core1BlitActive = true;
    if (core1BlitPaused) {
        // core0 possiede il display: solo pump audio (I2S non passa dal
        // bus SPI del TFT, puo' continuare senza limiti).
        core1BlitActive = false;
        pumpAudio(512);
        return;
    }

    if (frameReady) {
        uint16_t* src = (uint16_t*)displayFramePtr;
        int offY = (DISPLAY_H - LYNX_H * 2) / 2;

        for (int y = 0; y < DISPLAY_H; y += STRIP_H) {
            uint16_t* outRow = stripBuf;

            for (int dy = 0; dy < STRIP_H; dy++) {
                int actualY = y + dy;
                int srcY = (actualY - offY) / 2;

                if (srcY >= 0 && srcY < LYNX_H) {
                    uint16_t* srcRow = src + srcY * LYNX_W;
                    uint32_t* out32 = (uint32_t*)outRow;
                    for (int x = 0; x < LYNX_W; x++) {
                        uint32_t c = srcRow[x];
                        uint32_t c32 = (c << 16) | c;
                        *out32++ = c32;
                    }
                } else {
                    memset(outRow, 0, DISPLAY_W * 2);
                }
                outRow += DISPLAY_W;
            }

            tft.pushImageDMA(0, y, DISPLAY_W, STRIP_H, stripBuf);
            // pompa MENTRE la strip viaggia sul bus SPI.
            // Finestra cieca = 1 strip (~2,5ms) invece dell'intero
            // blit (~15-31ms). Il CAP (AUDIO_STRIP_PUMP_CAP) evita che
            // un backlog sequestri il blit: lo stallo cade comunque
            // DENTRO il trasferimento SPI in corso.
            pumpAudio(AUDIO_STRIP_PUMP_CAP);
            tft.dmaWait();
        }
        frameReady = false;
        core1BlitActive = false;
    } else {
        core1BlitActive = false;
        // Niente blit in corso: il pump gira da "timer". Se la lib e'
        // piena e il ring ha ancora campioni, write16 blocca ~45us per
        // campione: core1 si autoclocka al ritmo del consumo audio.
        // Se il ring e' vuoto esce subito (non e' un busy-loop).
        pumpAudio(512);
    }
}
