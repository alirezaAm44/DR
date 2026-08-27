/**
 * @file Result.h
 * @brief Lightweight Result<T, Error> pattern for embedded / Arduino environments.
 *
 * No exceptions, no std::expected, no dynamic allocation.
 */

#ifndef DJI_RONIN_RESULT_H
#define DJI_RONIN_RESULT_H

#include "RoninEnums.h"

namespace dji {
namespace ronin {

/**
 * Simple Result type.
 * For void success use Result<void, Error> specialization.
 */
template <typename T>
class Result {
public:
    static Result success(const T& value) {
        Result r;
        r.ok_ = true;
        r.value_ = value;
        r.error_ = Error::Ok;
        return r;
    }

    static Result failure(Error err) {
        Result r;
        r.ok_ = false;
        r.error_ = err;
        return r;
    }

    bool isOk() const { return ok_; }
    bool isError() const { return !ok_; }

    const T& value() const { return value_; }
    T& value() { return value_; }

    Error error() const { return error_; }

    explicit operator bool() const { return ok_; }

private:
    bool ok_;
    T value_;
    Error error_;
};

/** Specialization for void */
template <>
class Result<void> {
public:
    static Result success() {
        Result r;
        r.ok_ = true;
        r.error_ = Error::Ok;
        return r;
    }

    static Result failure(Error err) {
        Result r;
        r.ok_ = false;
        r.error_ = err;
        return r;
    }

    bool isOk() const { return ok_; }
    bool isError() const { return !ok_; }

    Error error() const { return error_; }

    explicit operator bool() const { return ok_; }

private:
    bool ok_;
    Error error_;
};

} // namespace ronin
} // namespace dji

#endif // DJI_RONIN_RESULT_H
