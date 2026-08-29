#include "SBUSNanoTx.h"

#if defined(ARDUINO_ARCH_AVR) && defined(__AVR_ATmega328P__)

namespace {
volatile uint8_t txFrame[25];
volatile uint8_t txIndex = 0;
volatile uint8_t txBit = 0;
volatile bool txBusy = false;
volatile uint32_t txCount = 0;
volatile uint32_t txDropped = 0;
bool txInverted = false;

inline void setWire(bool logicalHigh) {
    bool physicalHigh = txInverted ? !logicalHigh : logicalHigh;
    if (physicalHigh) PORTB |= _BV(PORTB0);       // D8
    else PORTB &= (uint8_t)~_BV(PORTB0);
}

void packFrame(const uint16_t ch[16], bool ch17, bool ch18,
               bool lost, bool failsafe) {
    for (uint8_t i = 0; i < 25; ++i) txFrame[i] = 0;
    txFrame[0] = 0x0F;

    uint32_t bitBuf = 0;
    uint8_t bits = 0;
    uint8_t out = 1;
    for (uint8_t c = 0; c < 16; ++c) {
        bitBuf |= ((uint32_t)(ch[c] & 0x07FFU)) << bits;
        bits += 11;
        while (bits >= 8) {
            txFrame[out++] = (uint8_t)(bitBuf & 0xFFU);
            bitBuf >>= 8;
            bits -= 8;
        }
    }
    txFrame[23] = (uint8_t)((ch17 ? 0x01 : 0) |
                            (ch18 ? 0x02 : 0) |
                            (lost ? 0x04 : 0) |
                            (failsafe ? 0x08 : 0));
    txFrame[24] = 0x00;
}

ISR(TIMER2_COMPA_vect) {
    // One interrupt per bit at exactly 100 kbit/s (16 MHz / 8 / 20).
    if (!txBusy) {
        setWire(false); // idle state
        return;
    }

    uint8_t byteIndex = txIndex;
    uint8_t b = txFrame[byteIndex];

    // 8E2 UART frame: start(0), 8 data LSB-first, even parity, 2 stop(1).
    if (txBit == 0) {
        setWire(false);
    } else if (txBit <= 8) {
        setWire((b >> (txBit - 1)) & 0x01);
    } else if (txBit == 9) {
        uint8_t ones = 0;
        uint8_t x = b;
        for (uint8_t i = 0; i < 8; ++i) { ones += x & 1U; x >>= 1; }
        setWire((ones & 1U) ? 1 : 0); // even parity
    } else {
        setWire(true);
    }

    ++txBit;
    if (txBit >= 12) {
        txBit = 0;
        ++txIndex;
        if (txIndex >= 25) {
            txIndex = 0;
            txBusy = false;
            ++txCount;
            setWire(false);
        }
    }
}
} // namespace

SBUSNanoTx::SBUSNanoTx(bool inverted) : inverted_(inverted) {}

bool SBUSNanoTx::begin() {
    txInverted = inverted_;
    DDRB |= _BV(DDB0); // D8 output
    setWire(false);

    cli();
    TCCR2A = _BV(WGM21);                 // CTC
    TCCR2B = _BV(CS21);                  // clk/8
    OCR2A = 19;                          // 16MHz/8/(19+1)=100kHz
    TCNT2 = 0;
    TIMSK2 |= _BV(OCIE2A);
    sei();
    return true;
}

bool SBUSNanoTx::write(const uint16_t channels[CHANNELS], bool ch17,
                       bool ch18, bool frameLost, bool failsafe) {
    if (txBusy) {
        ++txDropped;
        return false;
    }
    uint8_t sreg = SREG;
    cli();
    packFrame(channels, ch17, ch18, frameLost, failsafe);
    txIndex = 0;
    txBit = 0;
    txBusy = true;
    SREG = sreg;
    return true;
}

bool SBUSNanoTx::busy() const { return txBusy; }
uint32_t SBUSNanoTx::framesSent() const { return txCount; }
uint32_t SBUSNanoTx::droppedFrames() const { return txDropped; }

#else

SBUSNanoTx::SBUSNanoTx(bool inverted) : inverted_(inverted) {}
bool SBUSNanoTx::begin() { return false; }
bool SBUSNanoTx::write(const uint16_t[CHANNELS], bool, bool, bool, bool) { return false; }
bool SBUSNanoTx::busy() const { return false; }
uint32_t SBUSNanoTx::framesSent() const { return 0; }
uint32_t SBUSNanoTx::droppedFrames() const { return 0; }

#endif
