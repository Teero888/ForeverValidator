#include "engine/core/mw_cmd_buffer_core.h"
#include <new>
namespace {

constexpr long long MinHighFrequencyRunIntervalNanoseconds = 10000000ll;

}  // namespace

#if defined(__ELF__) && (defined(__GNUC__) || defined(__clang__))
thread_local __attribute__((tls_model("initial-exec")))
        CMwCmdBufferCore *CMwCmdBufferCore::TheCoreCmdBuffer = nullptr;
#else
thread_local CMwCmdBufferCore *CMwCmdBufferCore::TheCoreCmdBuffer = nullptr;
#endif

void CMwCmdBufferCore::HighFrequencyState::Reset(void) {
    lastRunStamp.reset();
    safeSectionDepth = 0ul;
    callsSinceRun = 0ul;
    running = false;
}

CMwCmdBufferCore::CMwCmdBufferCore(void) = default;

CMwCmdBufferCore::~CMwCmdBufferCore(void) {
    if (TheCoreCmdBuffer == this) {
        TheCoreCmdBuffer = nullptr;
    }
}

CMwCmdBufferCore *CMwCmdBufferCore::Current(void) {
    return TheCoreCmdBuffer;
}

void CMwCmdBufferCore::InstallAsCurrent(void) {
    TheCoreCmdBuffer = this;
}

void CMwCmdBufferCore::RestoreCurrent(CMwCmdBufferCore *core) noexcept {
    TheCoreCmdBuffer = core;
}

void CMwCmdBufferCore::Reset(void) {
    timer_.Reset();
    schemeBuffers_.clear();
    highFrequencyCommands_.Clear();
    highFrequency_.Reset();
}

void CMwCmdBufferCore::SetSimulationTime(
        unsigned long schemePeriod,
        unsigned long tickTime) {
    timer_.ConfigureSimulation(schemePeriod, tickTime);
}

CMwTimerAdapter &CMwCmdBufferCore::Timer(void) {
    return timer_;
}

const CMwTimerAdapter &CMwCmdBufferCore::Timer(void) const {
    return timer_;
}

CMwCmdBuffer &CMwCmdBufferCore::CommandBufferForScheme(
        unsigned long schemeLocation) {
    std::unique_ptr<CMwCmdBuffer> &buffer = schemeBuffers_[schemeLocation];
    if (!buffer) {
        buffer = std::make_unique<CMwCmdBuffer>();
    }
    return *buffer;
}

CMwCmdBuffer &CMwCmdBufferCore::HighFrequencyCommandBuffer(void) {
    return highFrequencyCommands_;
}

void CMwCmdBufferCore::HighFrequencyRun(unsigned long period) {
    if (highFrequency_.safeSectionDepth == 0ul ||
        highFrequency_.running ||
        highFrequencyCommands_.Empty()) {
        return;
    }

    ++highFrequency_.callsSinceRun;
    const unsigned long requiredCalls = period != 0ul ? period : 1ul;
    if (highFrequency_.callsSinceRun < requiredCalls) {
        return;
    }
    highFrequency_.callsSinceRun = 0ul;

    long long now = 0ll;
    CMwProfiler_GetTimeStamp_MThreadSafe(now);
    if (highFrequency_.lastRunStamp.has_value() &&
        now - *highFrequency_.lastRunStamp <=
                MinHighFrequencyRunIntervalNanoseconds) {
        return;
    }

    highFrequency_.lastRunStamp = now;
    highFrequency_.running = true;
    try {
        highFrequencyCommands_.Run(0ul);
    } catch (const std::bad_alloc &) {
        highFrequency_.running = false;
        throw;
    }
    highFrequency_.running = false;
}

void CMwCmdBufferCore::HighFrequencyEnterSafeSection(unsigned long period) {
    ++highFrequency_.safeSectionDepth;
    HighFrequencyRun(period);
}

void CMwCmdBufferCore::HighFrequencyLeaveSafeSection(void) {
    if (highFrequency_.safeSectionDepth != 0ul) {
        --highFrequency_.safeSectionDepth;
    }
}

void CMwCmdBufferCore::HighFrequencyYield(unsigned long period) {
    CMwCmdBufferCore *core = Current();
    if (core == nullptr) {
        return;
    }
    core->HighFrequencyEnterSafeSection(period);
    core->HighFrequencyLeaveSafeSection();
}
