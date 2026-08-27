/**
 * @file MockCANDriver.h
 * @brief Desktop / unit-test CAN driver that prints frames and can inject replies.
 */

#ifndef DJI_RONIN_MOCK_CAN_DRIVER_H
#define DJI_RONIN_MOCK_CAN_DRIVER_H

#include "../interfaces/ICANDriver.h"
#include <stdio.h>
#include <string.h>

namespace dji {
namespace ronin {

class MockCANDriver : public ICANDriver {
public:
    MockCANDriver() : hasPending_(false), verbose_(true) {}

    void setVerbose(bool v) { verbose_ = v; }

    Result<void> begin() override {
        if (verbose_) {
            printf("[MockCAN] begin()\n");
        }
        return Result<void>::success();
    }

    Result<void> send(const CANFrame& frame) override {
        if (verbose_) {
            printf("[MockCAN] TX  id=0x%03X  len=%u  data=", frame.id, frame.length);
            for (uint8_t i = 0; i < frame.length; ++i) {
                printf("%02X ", frame.data[i]);
            }
            printf("\n");
        }
        lastSent_ = frame;
        return Result<void>::success();
    }

    Result<void> receive(CANFrame& frame) override {
        if (!hasPending_) {
            return Result<void>::failure(Error::NoData);
        }
        frame = pending_;
        hasPending_ = false;
        if (verbose_) {
            printf("[MockCAN] RX  id=0x%03X  len=%u  data=", frame.id, frame.length);
            for (uint8_t i = 0; i < frame.length; ++i) {
                printf("%02X ", frame.data[i]);
            }
            printf("\n");
        }
        return Result<void>::success();
    }

    /**
     * Inject a frame that will be returned by the next receive() call.
     * Useful for unit tests that simulate Ronin replies.
     */
    void inject(const CANFrame& frame) {
        pending_ = frame;
        hasPending_ = true;
    }

    /**
     * Inject raw bytes as a CAN frame (id defaults to CAN_ID_RX).
     */
    void inject(uint32_t id, const uint8_t* data, uint8_t len) {
        CANFrame f;
        f.id = id;
        f.length = (len > 8) ? 8 : len;
        f.extended = false;
        for (uint8_t i = 0; i < f.length; ++i) {
            f.data[i] = data[i];
        }
        inject(f);
    }

    const CANFrame& lastSent() const { return lastSent_; }

private:
    CANFrame lastSent_;
    CANFrame pending_;
    bool hasPending_;
    bool verbose_;
};

} // namespace ronin
} // namespace dji

#endif // DJI_RONIN_MOCK_CAN_DRIVER_H
