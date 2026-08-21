#ifndef TMNF_REPLAY_DETERMINISTIC_EXECUTION_H
#define TMNF_REPLAY_DETERMINISTIC_EXECUTION_H

#include <cfenv>
#include <cstdint>

class CMwCmdBufferCore;

namespace tmnf::simulation {

class DeterministicExecutionScope {
public:
    DeterministicExecutionScope();
    ~DeterministicExecutionScope() {
        if (!restored_) {
            Restore();
        }
    }

    DeterministicExecutionScope(const DeterministicExecutionScope &) = delete;
    DeterministicExecutionScope &operator=(
            const DeterministicExecutionScope &) = delete;

    bool Established() const noexcept {
        return established_ && !restored_;
    }
    bool Restore() noexcept;
    static bool IsActive() noexcept;

private:
    std::fenv_t savedEnvironment_{};
    CMwCmdBufferCore *savedCommandBuffer_ = nullptr;
    std::uint32_t savedRandomState_ = 1u;
#if defined(__i386__) || defined(__x86_64__)
    unsigned int savedMxcsr_ = 0u;
#endif
    bool environmentCaptured_ = false;
    // Whether savedEnvironment_ holds anything. False on the common path, where
    // the caller was already rounding to nearest and the constructor left the
    // long-double environment alone rather than saving it.
    bool environmentRestorable_ = false;
    bool established_ = false;
    bool restored_ = false;
    bool ownsActiveScope_ = false;
};

}  // namespace tmnf::simulation

#endif
