/*
 * Questo file fa parte del progetto PiLynx.
 *
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 *
 * Ring buffer lock-free single-producer/single-consumer.
 *
 * Sicuro fra due core (RP2040/RP2350, o qualsiasi coppia produttore/
 * consumatore) SENZA mutex: la garanzia regge finche' un solo lato
 * scrive sempre e solo "head" (push) e l'altro scrive sempre e solo
 * "tail" (pop). Ogni lato legge la variabile dell'altro solo per
 * controllo, mai la scrive.
 *
 * CAPACITY deve essere una potenza di 2 (si usa una maschera invece
 * del modulo, piu' economico su Cortex-M0+/M33).
 */

#ifndef RING_BUFFER_SPSC_H
#define RING_BUFFER_SPSC_H

#include <stdint.h>

template <typename T, uint32_t CAPACITY>
class RingBufferSPSC {
    static_assert((CAPACITY & (CAPACITY - 1)) == 0, "CAPACITY deve essere una potenza di 2");

public:
    RingBufferSPSC() : head(0), tail(0) {}

    // Lato produttore. Ritorna false se il buffer e' pieno (il
    // chiamante decide se aspettare, scartare, o sovrascrivere).
    bool push(const T& value) {
        uint32_t h = head;
        uint32_t nextH = (h + 1) & MASK;
        if (nextH == tail) return false; // pieno
        buf[h] = value;
        __sync_synchronize(); // il dato deve essere visibile PRIMA di pubblicare head
        head = nextH;
        return true;
    }

    // Lato consumatore. Ritorna false se il buffer e' vuoto.
    bool pop(T& outValue) {
        uint32_t t = tail;
        if (t == head) return false; // vuoto
        outValue = buf[t];
        __sync_synchronize();
        tail = (t + 1) & MASK;
        return true;
    }

    bool isFull() const { return ((head + 1) & MASK) == tail; }
    bool isEmpty() const { return head == tail; }
    uint32_t available() const { return (head - tail) & MASK; }   // quanti pronti da leggere
    uint32_t freeSpace() const { return (tail - head - 1) & MASK; } // quanti ancora scrivibili

private:
    static const uint32_t MASK = CAPACITY - 1;
    T buf[CAPACITY];
    volatile uint32_t head; // scritto SOLO dal produttore
    volatile uint32_t tail; // scritto SOLO dal consumatore
};

#endif
