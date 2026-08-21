#pragma once

#include <map>
#include <memory>
#include <optional>

#include "engine/core/mw_cmd.h"
#include "engine/core/mw_timer.h"
class CMwCmdBufferCore : public CMwNod {
public:
    CMwCmdBufferCore(void);
    ~CMwCmdBufferCore(void) override;

    static CMwCmdBufferCore *Current(void);
    void InstallAsCurrent(void);
    static void RestoreCurrent(CMwCmdBufferCore *core) noexcept;

    void Reset(void);
    void SetSimulationTime(unsigned long schemePeriod,
                           unsigned long tickTime);

    void HighFrequencyRun(unsigned long period);
    void HighFrequencyEnterSafeSection(unsigned long period);
    void HighFrequencyLeaveSafeSection(void);
    void HighFrequencyYield(unsigned long period);

    CMwTimerAdapter &Timer(void);
    const CMwTimerAdapter &Timer(void) const;
    CMwCmdBuffer &CommandBufferForScheme(unsigned long schemeLocation);
    CMwCmdBuffer &HighFrequencyCommandBuffer(void);

private:
    struct HighFrequencyState {
        std::optional<long long> lastRunStamp;
        unsigned long safeSectionDepth = 0ul;
        unsigned long callsSinceRun = 0ul;
        bool running = false;

        void Reset(void);
    };

#if defined(__ELF__) && (defined(__GNUC__) || defined(__clang__))
    static thread_local __attribute__((tls_model("initial-exec")))
            CMwCmdBufferCore *TheCoreCmdBuffer;
#else
    static thread_local CMwCmdBufferCore *TheCoreCmdBuffer;
#endif

    CMwTimerAdapter timer_;
    std::map<unsigned long, std::unique_ptr<CMwCmdBuffer>> schemeBuffers_;
    CMwCmdBuffer highFrequencyCommands_;
    HighFrequencyState highFrequency_;
};
