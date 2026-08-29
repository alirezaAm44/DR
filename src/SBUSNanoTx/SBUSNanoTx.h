#pragma once

#include <Arduino.h>

/**
 * SBUS transmitter for classic Arduino Nano / Uno (ATmega328P).
 *
 * Uses Timer2 CTC at 100 kHz and does not consume the hardware UART,
 * so Serial Monitor remains available. Output pin is D8 (PB0).
 *
 * SBUS wire polarity can be selected in software. The Ronin software can
 * therefore be configured for the same polarity; no external inverter is
 * required for the Ronin input when it accepts the selected polarity.
 *
 * Frame: 25 bytes, 16 x 11-bit channels, 8E2, 100000 baud.
 */
class SBUSNanoTx {
public:
    static constexpr uint8_t CHANNELS = 16;
    static constexpr uint16_t MIN_VALUE = 172;
    static constexpr uint16_t MID_VALUE = 992;
    static constexpr uint16_t MAX_VALUE = 1811;

    explicit SBUSNanoTx(bool inverted = false);

    // Initializes Timer2 and D8. Must be called once from setup().
    bool begin();

    // Copies 16 channel values and starts one frame if the transmitter is idle.
    // Returns false if a previous frame is still being transmitted.
    bool write(const uint16_t channels[CHANNELS], bool ch17 = false,
               bool ch18 = false, bool frameLost = false,
               bool failsafe = false);

    bool busy() const;
    uint32_t framesSent() const;
    uint32_t droppedFrames() const;

private:
    bool inverted_;
};
