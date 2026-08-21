#pragma once

#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include "engine/resources/asset_audience.h"
#include "engine/core/engine_types.h"
#include "engine/core/cfast_buffer.h"
#include "engine/core/mw_id.h"
struct CFastString;
struct CMwNod;
struct CSystemFid;
struct CSystemFidFile;
struct CSystemFids;
struct CSystemPackDesc;

template <typename T>
class CMwNodRef;

struct CSystemFidParameters {
    struct SParam;
    struct SParam_Id;
    struct SParam_Set;
    struct SParam_Fid_Common;
    struct SParam_Fid;
    struct SParam_Fids;
    class Scope;

    static constexpr u32 InclusiveDefaultRemapLimit = 100u;
    static constexpr unsigned long DefaultPackElemIndex =
            std::numeric_limits<u32>::max();

    static const CSystemFidParameters None;
    static const CSystemFidParameters Empty;

    CSystemFidParameters(void);
    CSystemFidParameters(const CSystemFidParameters &other);
    CSystemFidParameters &operator=(const CSystemFidParameters &other);
    virtual ~CSystemFidParameters(void);

    void ResetAllParameters(void);
    void SetInclusiveDefaultsOnly(void);
    bool HasNonDefaultParameters(void) const;
    void VisitDirectFidRemaps(
            const std::function<void(const SParam_Fid &)> &visitor) const;
    int CanCollectInclusiveDefaultRemap(void) const;
    void DisableInclusiveDefaultRemap(void);
    void AddAllParamsFrom(const CSystemFidParameters &source);
    int MergeForClassIds(
            const CSystemFidParameters &source,
            const CFastBuffer<CSystemFid *> &fids);
    int MergeForAudience(
            const CSystemFidParameters &source,
            const AssetAudience &audience);
    int MergeForFid(
            const CSystemFidParameters &source,
            CSystemFid *fid);
    void AddTemporaryFidRemap(CSystemFid *fid,
                              CSystemPackDesc *packDesc,
                              const CFastString *prefix,
                              unsigned long id);
    void AddTemporaryFidsRemap(CSystemFids *fids,
                               CSystemPackDesc *packDesc,
                               const CFastString *prefix,
                               unsigned long id,
                               int ensureTrailingBackslash);
    void AddDefaultFidRemap(CSystemFid *fid);
    void AddDefaultFidsRemap(CSystemFids *fids);
    void CollectDefaultFidsRemapsFor(CSystemFid *loadableFid);
    void CollectDefaultFidOrDisable(CSystemFid *fid);
    void CollectDefaultFidsOrDisable(CSystemFid *fid);
    int AddParam(const SParam &param);
    int GetParamValue(SParam &param) const;
    unsigned long GetParamIdValue(const CMwId &id) const;

protected:
    void DoOneRemap(
            CSystemFid *fid,
            const CSystemPackDesc *&outPackDesc,
            CFastString &outPackElemName,
            unsigned long &outPackElemIndex,
            CSystemFidParameters &collectedParams) const;
public:
    void Remap(
            CSystemFid *&sourceFid,
            CSystemFid *&outFid,
            CSystemFidParameters &collectedParams,
            CMwNod *owner) const;
    static void Pop(const CSystemFidParameters *params);
    static void StaticRelease(void)
#if defined(__GNUC__) || defined(__clang__)
            __attribute__((cold, noinline))
#endif
            ;
    static std::size_t ActiveScopeCount(void) noexcept;
    static const CSystemFidParameters &GetCurrentParameters();
    static void Push(
            const CSystemFidParameters *params,
            int mergeWithPrevious);
    static int RemappedLoadFromFid(
            CMwNod *&outNod,
            CSystemFid *fid,
            CMwNod *owner);
    template <typename T>
    static int RemappedLoadFromFid(
            CMwNodRef<T> &outNod,
            CSystemFid *fid,
            CMwNod *owner);
    static int RemappedLoadFromFid_ParamFid(
            CMwNod *&outNod,
            CSystemFid *fid,
            CSystemFid *paramFid,
            CSystemFidFile *fidFile);
    static void DumpUsageStatistics(void);

private:
    friend struct CSystemFid;

    explicit CSystemFidParameters(int inclusiveFlag);
    int IncludesFid(
            const CSystemFidParameters &other,
            CSystemFid *fid,
            int skipOtherOptionalCheck) const;
    int IncludesForAudience(
            const CSystemFidParameters &other,
            const AssetAudience &audience,
            int skipOtherOptionalCheck) const;
    int IncludesWhere(
            const CSystemFidParameters &other,
            const std::function<bool(const SParam &)> &matches,
            int skipOtherOptionalCheck) const;
    void SimplifyWhere(
            const std::function<bool(const SParam &)> &matches,
            bool hasNoRuntimeParameters);
    void SimplifyForFid(CSystemFid *fid);
    void SimplifyForAudience(const AssetAudience &audience);
    static void PushForAudience(
            CSystemFid *fid,
            const AssetAudience &audience);
    static void PopForTarget(CSystemFid *fid);
    static void PushForTarget(CSystemFid *fid);
    std::vector<std::unique_ptr<SParam>> parameters;
    bool inclusive;
};

class CSystemFidParameters::Scope {
public:
    Scope(const CSystemFidParameters &parameters, int mergeWithPrevious);
    explicit Scope(CSystemFid *fid);
    ~Scope(void);

    Scope(const Scope &) = delete;
    Scope &operator=(const Scope &) = delete;

private:
    const CSystemFidParameters *ownedFrame;
};
