#include "engine/resources/system_fid_parameters.h"
#include "engine/resources/system_fid_parameter_types.h"
#include <algorithm>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "engine/core/cmw_nod.h"
#include "engine/resources/system_engine.h"
#include "engine/resources/system_fid.h"
#include "engine/resources/system_fid_file.h"
#include "engine/resources/system_fids.h"
#include "engine/resources/system_file_name.h"
#include "engine/resources/system_pack_desc.h"
#include "engine/resources/system_pack_manager.h"
namespace {

using CSystemFidParameterFrame = std::unique_ptr<CSystemFidParameters>;
using CSystemFidParameterFrameStack =
        std::vector<CSystemFidParameterFrame>;

CSystemFidParameterFrameStack &CSystemFidParameterFrames(void) {
#if defined(__ELF__) && (defined(__GNUC__) || defined(__clang__))
    thread_local __attribute__((tls_model("initial-exec")))
            CSystemFidParameterFrameStack frames;
#else
    thread_local CSystemFidParameterFrameStack frames;
#endif
    return frames;
}

const CSystemFidParameters *CurrentFidParameterFrame(void) {
    const auto &frames = CSystemFidParameterFrames();
    return frames.empty() ? nullptr : frames.back().get();
}

void PopOwnedFidParameterFrame(
        const CSystemFidParameters *ownedFrame) noexcept {
    if (ownedFrame == nullptr) {
        return;
    }
    auto &frames = CSystemFidParameterFrames();
    if (frames.empty() || frames.back().get() != ownedFrame) {
        std::abort();
    }
    frames.pop_back();
}

struct ParameterComparison {
    bool sameValue;
    bool differentKey;

    bool HasSameKey(void) const {
        return !differentKey;
    }
};

ParameterComparison CompareParameters(
        const CSystemFidParameters::SParam &existing,
        const CSystemFidParameters::SParam &incoming) {
    int sameValue = 0;
    int differentKey = 0;
    existing.Compare(incoming, sameValue, differentKey);
    return {sameValue != 0, differentKey != 0};
}

using OwnedParameter =
        std::unique_ptr<CSystemFidParameters::SParam>;
using ParameterCollection = std::vector<OwnedParameter>;

OwnedParameter DuplicateParameter(
        const CSystemFidParameters::SParam &parameter) {
    return parameter.Clone();
}

ParameterCollection CloneParameters(
        const ParameterCollection &source) {
    ParameterCollection copy;
    copy.reserve(source.size());
    for (const OwnedParameter &parameter : source) {
        copy.push_back(DuplicateParameter(*parameter));
    }
    return copy;
}

int StoreParameter(
        ParameterCollection &parameters,
        const CSystemFidParameters::SParam &parameter) {
    for (OwnedParameter &existing : parameters) {
        const ParameterComparison comparison =
                CompareParameters(*existing, parameter);
        if (!comparison.HasSameKey()) {
            continue;
        }
        existing->CopyFrom(parameter);
        return comparison.sameValue ? 0 : 1;
    }

    parameters.push_back(DuplicateParameter(parameter));
    return 1;
}

}  // namespace

CSystemFidParameters::Scope::Scope(
        const CSystemFidParameters &parameters,
        int mergeWithPrevious)
        : ownedFrame(nullptr) {
    CSystemFidParameters::Push(&parameters, mergeWithPrevious);
    ownedFrame = CurrentFidParameterFrame();
}

CSystemFidParameters::Scope::Scope(
        CSystemFid *fid)
        : ownedFrame(nullptr) {
    const CSystemFidParameters *previousFrame = CurrentFidParameterFrame();
    CSystemFidParameters::PushForTarget(fid);
    const CSystemFidParameters *currentFrame = CurrentFidParameterFrame();
    if (currentFrame != previousFrame) {
        ownedFrame = currentFrame;
    }
}

CSystemFidParameters::Scope::~Scope(void) {
    PopOwnedFidParameterFrame(ownedFrame);
}

const CSystemFidParameters CSystemFidParameters::None(1);
const CSystemFidParameters CSystemFidParameters::Empty(0);

CSystemFidParameters::SParam::SParam(
        AssetAudience parameterAudience,
        int isOptional)
        : audience(std::move(parameterAudience)),
          optional(isOptional != 0) {}

CSystemFidParameters::SParam::~SParam(void) = default;

std::unique_ptr<CSystemFidParameters::SParam>
CSystemFidParameters::SParam::Clone(void) const {
    std::unique_ptr<SParam> clone(Duplicate());
    if (clone == nullptr) {
        throw std::logic_error("parameter duplication returned null");
    }
    return clone;
}

void CSystemFidParameters::SParam::CopyFrom(
        const CSystemFidParameters::SParam &source) {
    if (this == &source) {
        return;
    }
    audience = source.audience;
    optional = source.optional;
}

void CSystemFidParameters::SParam::StoreCompareResult(
        int sameValue,
        int differentKey,
        int &sameValueOut,
        int &differentKeyOut) {
    sameValueOut = differentKey == 0 && sameValue != 0;
    differentKeyOut = differentKey != 0;
}

int CSystemFidParameters::SParam::IsOptional(void) const {
    return optional;
}

int CSystemFidParameters::SParam::MatchesAudience(
        const AssetAudience &candidateAudience) const {
    return audience.Overlaps(candidateAudience) ? 1 : 0;
}

int CSystemFidParameters::SParam::MatchesFid(CSystemFid &fid) const {
    return audience.Accepts(fid) ? 1 : 0;
}

int CSystemFidParameters::SParam::MatchesAnyFids(
        const CFastBuffer<CSystemFid *> &fids) const {
    if (fids.empty()) {
        return 1;
    }
    for (CSystemFid *fid : fids) {
        if (fid != nullptr && MatchesFid(*fid) != 0) {
            return 1;
        }
    }
    return 0;
}

CSystemFidParameters::SParam_Id::SParam_Id(void)
        : SParam(AnyAssetAudience(), 0),
          id(),
          value(0u) {}

CSystemFidParameters::SParam_Id::SParam_Id(
        const CMwId &id,
        unsigned long value,
        AssetAudience parameterAudience,
        int isOptional)
        : SParam(std::move(parameterAudience), isOptional),
          id(id),
          value(value) {}

int CSystemFidParameters::SParam_Id::IsDefault(void) const {
    return value == 0u;
}

CSystemFidParameters::SParam *
CSystemFidParameters::SParam_Id::Duplicate(void) const {
    return std::make_unique<SParam_Id>(*this).release();
}

void CSystemFidParameters::SParam_Id::CopyFrom(
        const CSystemFidParameters::SParam &source) {
    if (this == &source) {
        return;
    }
    const auto *typed = dynamic_cast<const SParam_Id *>(&source);
    if (typed == nullptr) {
        return;
    }
    SParam::CopyFrom(*typed);
    id = typed->id;
    value = typed->value;
}

void CSystemFidParameters::SParam_Id::Compare(
        const CSystemFidParameters::SParam &candidate,
        int &sameValueOut,
        int &differentKeyOut) const {
    const auto *typed = dynamic_cast<const SParam_Id *>(&candidate);
    if (typed == nullptr) {
        StoreCompareResult(0, 1, sameValueOut, differentKeyOut);
        return;
    }
    const int differentKey = id != typed->id;
    const int sameValue = differentKey == 0 && value == typed->value;
    StoreCompareResult(
            sameValue,
            differentKey,
            sameValueOut,
            differentKeyOut);
}

CSystemFidParameters::SParam_Set::SParam_Set(void)
        : SParam(AnyAssetAudience(), 1),
          fid(nullptr),
          paramName(),
          value() {}

CSystemFidParameters::SParam_Set::SParam_Set(
        CSystemFid *fid,
        const CFastString &paramName,
        const CFastStringInt &value,
        AssetAudience parameterAudience)
        : SParam(std::move(parameterAudience), 1),
          fid(fid),
          paramName(paramName),
          value(value) {}

CSystemFidParameters::SParam_Set::~SParam_Set(void) = default;

int CSystemFidParameters::SParam_Set::IsDefault(void) const {
    return paramName.Empty() || value.Empty();
}

CSystemFidParameters::SParam *
CSystemFidParameters::SParam_Set::Duplicate(void) const {
    return std::make_unique<SParam_Set>(*this).release();
}

void CSystemFidParameters::SParam_Set::CopyFrom(
        const CSystemFidParameters::SParam &source) {
    if (this == &source) {
        return;
    }
    const auto *typed = dynamic_cast<const SParam_Set *>(&source);
    if (typed == nullptr) {
        return;
    }
    SParam::CopyFrom(*typed);
    fid = typed->fid;
    paramName = typed->paramName;
    value = typed->value;
}

void CSystemFidParameters::SParam_Set::Compare(
        const CSystemFidParameters::SParam &candidate,
        int &sameValueOut,
        int &differentKeyOut) const {
    const auto *typed = dynamic_cast<const SParam_Set *>(&candidate);
    if (typed == nullptr) {
        StoreCompareResult(0, 1, sameValueOut, differentKeyOut);
        return;
    }
    const int differentKey = fid != typed->fid ||
                             paramName.String() != typed->paramName.String();
    const int sameValue = differentKey == 0 &&
                          value.String() == typed->value.String();
    StoreCompareResult(
            sameValue,
            differentKey,
            sameValueOut,
            differentKeyOut);
}

CSystemFidParameters::SParam_Fid_Common::SParam_Fid_Common(
        CSystemPackDesc *packDesc,
        const CFastString &prefix,
        unsigned long packElemIndex)
        : SParam(AnyAssetAudience(), 0),
          packDesc(packDesc),
          prefix(prefix),
          packElemIndex(packElemIndex) {}

CSystemFidParameters::SParam_Fid_Common::~SParam_Fid_Common(void) = default;

CSystemPackDesc *
CSystemFidParameters::SParam_Fid_Common::PackDesc(void) const {
    return packDesc.Get();
}

const CFastString &
CSystemFidParameters::SParam_Fid_Common::RemapPrefix(void) const {
    return prefix;
}

unsigned long
CSystemFidParameters::SParam_Fid_Common::PackElemIndexValue(void) const {
    return packElemIndex;
}

int CSystemFidParameters::SParam_Fid_Common::HasPackDesc(void) const {
    return static_cast<bool>(packDesc);
}

int CSystemFidParameters::SParam_Fid_Common::HasSpecialPackDesc(void) const {
    return packDesc && packDesc->IsChecksumSpecial() != 0;
}

void CSystemFidParameters::SParam_Fid_Common::StorePackTarget(
        const CSystemPackDesc *&outPackDesc,
        unsigned long &outPackElemIndex) const {
    outPackDesc = packDesc.Get();
    outPackElemIndex = packElemIndex;
}

void CSystemFidParameters::SParam_Fid_Common::StoreMatchedRemap(
        const CSystemPackDesc *&outPackDesc,
        CFastString &outPackElemName,
        unsigned long &outPackElemIndex) const {
    StorePackTarget(outPackDesc, outPackElemIndex);
    outPackElemName = prefix;
}

int CSystemFidParameters::SParam_Fid_Common::IsDefault(void) const {
    return !packDesc && prefix.Empty();
}

void CSystemFidParameters::SParam_Fid_Common::CopyFrom(
        const CSystemFidParameters::SParam &source) {
    if (this == &source) {
        return;
    }
    const auto *typed = dynamic_cast<const SParam_Fid_Common *>(&source);
    if (typed == nullptr) {
        return;
    }
    SParam::CopyFrom(*typed);
    packDesc = typed->packDesc;
    prefix = typed->prefix;
    packElemIndex = typed->packElemIndex;
}

void CSystemFidParameters::SParam_Fid_Common::CompareFidCommon(
        const CSystemFidParameters::SParam_Fid_Common &candidate,
        int targetMatches,
        int &sameValueOut,
        int &differentKeyOut) const {
    const int differentKey = targetMatches == 0;
    const int sameValue = differentKey == 0 &&
                          packDesc.Get() == candidate.packDesc.Get() &&
                          prefix.String() == candidate.prefix.String();
    StoreCompareResult(
            sameValue,
            differentKey,
            sameValueOut,
            differentKeyOut);
}

CSystemFidParameters::SParam_Fid::SParam_Fid(void)
        : SParam_Fid(nullptr) {}

CSystemFidParameters::SParam_Fid::SParam_Fid(CSystemFid *fid)
        : SParam_Fid(
                  fid,
                  nullptr,
                  CFastString(),
                  DefaultPackElemIndex) {}

CSystemFidParameters::SParam_Fid::SParam_Fid(
        CSystemFid *fid,
        CSystemPackDesc *packDesc,
        const CFastString &prefix,
        unsigned long packElemIndex)
        : SParam_Fid_Common(packDesc, prefix, packElemIndex),
          fid(fid) {}

CSystemFidParameters::SParam *
CSystemFidParameters::SParam_Fid::Duplicate(void) const {
    return std::make_unique<SParam_Fid>(*this).release();
}

void CSystemFidParameters::SParam_Fid::CopyFrom(
        const CSystemFidParameters::SParam &source) {
    if (this == &source) {
        return;
    }
    const auto *typed = dynamic_cast<const SParam_Fid *>(&source);
    if (typed == nullptr) {
        return;
    }
    SParam_Fid_Common::CopyFrom(*typed);
    fid = typed->fid;
}

void CSystemFidParameters::SParam_Fid::Compare(
        const CSystemFidParameters::SParam &candidate,
        int &sameValue,
        int &differentKey) const {
    const auto *typed = dynamic_cast<const SParam_Fid *>(&candidate);
    if (typed == nullptr) {
        StoreCompareResult(0, 1, sameValue, differentKey);
        return;
    }
    CompareFidCommon(*typed, fid == typed->fid, sameValue, differentKey);
}

int CSystemFidParameters::SParam_Fid::TargetsFid(
        const CSystemFid *loadableFid) const {
    return fid == loadableFid;
}

CSystemFidParameters::SParam_Fids::SParam_Fids(void)
        : SParam_Fids(nullptr) {}

CSystemFidParameters::SParam_Fids::SParam_Fids(CSystemFids *fids)
        : SParam_Fids(
                  fids,
                  nullptr,
                  CFastString(),
                  DefaultPackElemIndex) {}

CSystemFidParameters::SParam_Fids::SParam_Fids(
        CSystemFids *fids,
        CSystemPackDesc *packDesc,
        const CFastString &prefix,
        unsigned long packElemIndex)
        : SParam_Fid_Common(packDesc, prefix, packElemIndex),
          fids(fids) {}

CSystemFidParameters::SParam *
CSystemFidParameters::SParam_Fids::Duplicate(void) const {
    return std::make_unique<SParam_Fids>(*this).release();
}

void CSystemFidParameters::SParam_Fids::CopyFrom(
        const CSystemFidParameters::SParam &source) {
    if (this == &source) {
        return;
    }
    const auto *typed = dynamic_cast<const SParam_Fids *>(&source);
    if (typed == nullptr) {
        return;
    }
    SParam_Fid_Common::CopyFrom(*typed);
    fids = typed->fids;
}

void CSystemFidParameters::SParam_Fids::Compare(
        const CSystemFidParameters::SParam &candidate,
        int &sameValue,
        int &differentKey) const {
    const auto *typed = dynamic_cast<const SParam_Fids *>(&candidate);
    if (typed == nullptr) {
        StoreCompareResult(0, 1, sameValue, differentKey);
        return;
    }
    CompareFidCommon(*typed, fids == typed->fids, sameValue, differentKey);
}

int CSystemFidParameters::SParam_Fids::BuildFolderRemapName(
        CSystemFid *loadableFid,
        CFastString *outPackElemName) const {
    CSystemFidFile *fileFid = dynamic_cast<CSystemFidFile *>(loadableFid);
    if (fileFid == nullptr || fids == nullptr || outPackElemName == nullptr) {
        return 0;
    }

    CFastStringInt tempName;
    if (fileFid->GetFullNameUpTo(tempName, fids) == 0) {
        return 0;
    }

    CFastStringInt strippedName;
    CSystemFileName::StripExtension(tempName, strippedName);

    CFastStringInt prefixWide;
    const SStringParam prefixAnsi = {
        prefix.Data(),
        prefix.Length(),
    };
    prefixWide.SetString(prefixAnsi);

    SStringParamInt prefixParam = {
        prefixWide.Data(),
        prefixWide.Length(),
        0u,
    };
    strippedName.ConcatBefore(prefixParam);
    strippedName.GetAscii(*outPackElemName);
    return 1;
}

CSystemFidParameters::~CSystemFidParameters(void) = default;

CSystemFidParameters::CSystemFidParameters(int inclusiveFlag)
        : parameters(),
          inclusive(inclusiveFlag != 0) {}

CSystemFidParameters::CSystemFidParameters(void)
        : CSystemFidParameters(0) {}

void CSystemFidParameters::ResetAllParameters(void) {
    parameters.clear();
}

void CSystemFidParameters::SetInclusiveDefaultsOnly(void) {
    ResetAllParameters();
    inclusive = true;
}

bool CSystemFidParameters::HasNonDefaultParameters(void) const {
    return std::any_of(
            parameters.begin(),
            parameters.end(),
            [](const OwnedParameter &parameter) {
                return parameter->IsDefault() == 0;
            });
}

int CSystemFidParameters::IncludesFid(
        const CSystemFidParameters &other,
        CSystemFid *fid,
        int skipOtherOptionalCheck) const {
    return IncludesWhere(
            other,
            [fid](const SParam &param) {
                return fid == nullptr || param.MatchesFid(*fid) != 0;
            },
            skipOtherOptionalCheck);
}

int CSystemFidParameters::IncludesForAudience(
        const CSystemFidParameters &other,
        const AssetAudience &audience,
        int skipOtherOptionalCheck) const {
    return IncludesWhere(
            other,
            [&audience](const SParam &param) {
                return param.MatchesAudience(audience) != 0;
            },
            skipOtherOptionalCheck);
}

int CSystemFidParameters::IncludesWhere(
        const CSystemFidParameters &other,
        const std::function<bool(const SParam &)> &matches,
        int skipOtherOptionalCheck) const {
    if (!other.HasNonDefaultParameters() && !HasNonDefaultParameters()) {
        return 1;
    }

    std::vector<bool> usedParams(other.parameters.size(), false);
    for (const OwnedParameter &queryValue : parameters) {
        const SParam &queryParam = *queryValue;
        const bool classDoesNotMatch = !matches(queryParam);

        bool found = false;
        bool sameValue = false;
        for (size_t candidateIndex = 0u;
             candidateIndex < other.parameters.size();
             ++candidateIndex) {
            if (usedParams[candidateIndex]) {
                continue;
            }
            const ParameterComparison comparison = CompareParameters(
                    *other.parameters[candidateIndex],
                    queryParam);
            if (!comparison.HasSameKey()) {
                continue;
            }
            usedParams[candidateIndex] = true;
            found = true;
            sameValue = comparison.sameValue;
            break;
        }

        if (found) {
            if (!sameValue) {
                return 0;
            }
            continue;
        }
        if (classDoesNotMatch || queryParam.IsDefault() != 0) {
            continue;
        }
        if (queryParam.IsOptional() != 0 || !other.inclusive) {
            return 0;
        }
    }

    if (skipOtherOptionalCheck != 0) {
        return 1;
    }

    for (size_t index = 0u; index < other.parameters.size(); ++index) {
        if (!usedParams[index] &&
            other.parameters[index]->IsDefault() == 0) {
            return 0;
        }
    }
    return 1;
}

void CSystemFidParameters::SimplifyForAudience(
        const AssetAudience &audience) {
    SimplifyWhere(
            [&audience](const SParam &param) {
                return param.MatchesAudience(audience) != 0;
            },
            audience.IsExactly(AssetType::SceneEngine));
}

void CSystemFidParameters::SimplifyForFid(CSystemFid *fid) {
    SimplifyWhere(
            [fid](const SParam &param) {
                return fid == nullptr || param.MatchesFid(*fid) != 0;
            },
            fid != nullptr && fid->MatchesAudience(SceneEngineAudience()));
}

void CSystemFidParameters::SimplifyWhere(
        const std::function<bool(const SParam &)> &matches,
        bool hasNoRuntimeParameters) {
    if (hasNoRuntimeParameters) {
        SetInclusiveDefaultsOnly();
    }

    size_t index = 0u;
    while (index < parameters.size()) {
        if ((!inclusive && parameters[index]->IsDefault() != 0) ||
            !matches(*parameters[index])) {
            parameters.erase(parameters.begin() + index);
            continue;
        }

        size_t duplicateIndex = index + 1u;
        while (duplicateIndex < parameters.size()) {
            const ParameterComparison comparison = CompareParameters(
                    *parameters[duplicateIndex],
                    *parameters[index]);
            if (comparison.HasSameKey()) {
                parameters.erase(parameters.begin() + duplicateIndex);
            } else {
                ++duplicateIndex;
            }
        }
        ++index;
    }
}

int CSystemFidParameters::AddParam(
        const CSystemFidParameters::SParam &param) {
    return StoreParameter(parameters, param);
}

int CSystemFidParameters::GetParamValue(
        CSystemFidParameters::SParam &param) const {
    for (const OwnedParameter &candidate : parameters) {
        if (CompareParameters(*candidate, param).HasSameKey()) {
            param.CopyFrom(*candidate);
            return 1;
        }
    }
    return 0;
}

unsigned long CSystemFidParameters::GetParamIdValue(const CMwId &id) const {
    SParam_Id param(id, 0u, AnyAssetAudience(), 0);
    GetParamValue(param);
    return param.value;
}

void CSystemFidParameters::VisitDirectFidRemaps(
        const std::function<void(const SParam_Fid &)> &visitor) const {
    if (!visitor) {
        return;
    }
    for (const OwnedParameter &parameter : parameters) {
        const SParam_Fid *fidParameter =
                dynamic_cast<const SParam_Fid *>(parameter.get());
        if (fidParameter != nullptr) {
            visitor(*fidParameter);
        }
    }
}

int CSystemFidParameters::CanCollectInclusiveDefaultRemap(void) const {
    return inclusive && parameters.size() <= InclusiveDefaultRemapLimit;
}

void CSystemFidParameters::DisableInclusiveDefaultRemap(void) {
    inclusive = false;
}

namespace {

struct RemapOutput {
    const CSystemPackDesc *&packDesc;
    CFastString &packElemName;
    unsigned long &packElemIndex;

    void StoreDirectMatch(
            const CSystemFidParameters::SParam_Fid_Common &param) const {
        param.StoreMatchedRemap(packDesc, packElemName, packElemIndex);
    }

    void StorePackTarget(
            const CSystemFidParameters::SParam_Fid_Common &param) const {
        param.StorePackTarget(packDesc, packElemIndex);
    }

    void Clear(void) const {
        packDesc = nullptr;
        packElemIndex = CSystemFidParameters::DefaultPackElemIndex;
        packElemName.Assign({});
    }
};

}  // namespace

void CSystemFidParameters::AddDefaultFidRemap(CSystemFid *fid) {
    AddTemporaryFidRemap(
            fid,
            nullptr,
            &CFastString::s_Null,
            DefaultPackElemIndex);
}

void CSystemFidParameters::AddDefaultFidsRemap(CSystemFids *fids) {
    AddTemporaryFidsRemap(
            fids,
            nullptr,
            &CFastString::s_Null,
            DefaultPackElemIndex,
            0);
}

void CSystemFidParameters::CollectDefaultFidsRemapsFor(
        CSystemFid *loadableFid) {
    CSystemFids *owningFids = loadableFid != nullptr
            ? loadableFid->Location()
            : nullptr;
    if (owningFids == nullptr) {
        return;
    }
    AddDefaultFidsRemap(owningFids);

    CSystemFids *parentFids = owningFids->Parent();
    if (parentFids != nullptr &&
        dynamic_cast<CSystemFidsDrive *>(parentFids) == nullptr) {
        AddDefaultFidsRemap(parentFids);
    }
}

void CSystemFidParameters::CollectDefaultFidOrDisable(CSystemFid *fid) {
    if (CanCollectInclusiveDefaultRemap()) {
        AddDefaultFidRemap(fid);
    } else {
        DisableInclusiveDefaultRemap();
    }
}

void CSystemFidParameters::CollectDefaultFidsOrDisable(CSystemFid *fid) {
    if (CanCollectInclusiveDefaultRemap()) {
        CollectDefaultFidsRemapsFor(fid);
    } else {
        DisableInclusiveDefaultRemap();
    }
}

void CSystemFidParameters::DoOneRemap(
        CSystemFid *fid,
        const CSystemPackDesc *&outPackDesc,
        CFastString &outPackElemName,
        unsigned long &outPackElemIndex,
        CSystemFidParameters &collectedParams) const {
    CSystemFid *loadableFid = fid->ParametrizedGetLoadableFid();
    const RemapOutput out = {
        outPackDesc,
        outPackElemName,
        outPackElemIndex,
    };

    for (const OwnedParameter &parameter : parameters) {
        const SParam_Fid *fidParameter =
                dynamic_cast<const SParam_Fid *>(parameter.get());
        if (fidParameter != nullptr &&
            fidParameter->TargetsFid(loadableFid) != 0) {
            out.StoreDirectMatch(*fidParameter);
            collectedParams.AddParam(*fidParameter);
            return;
        }
    }
    collectedParams.CollectDefaultFidOrDisable(loadableFid);

    for (const OwnedParameter &parameter : parameters) {
        const SParam_Fids *fidsParameter =
                dynamic_cast<const SParam_Fids *>(parameter.get());
        if (fidsParameter != nullptr &&
            fidsParameter->BuildFolderRemapName(
                    loadableFid,
                    &out.packElemName) != 0) {
            out.StorePackTarget(*fidsParameter);
            collectedParams.AddParam(*fidsParameter);
            return;
        }
    }
    collectedParams.CollectDefaultFidsOrDisable(loadableFid);
    out.Clear();
}

void CSystemFidParameters::Remap(
        CSystemFid *&sourceFid,
        CSystemFid *&outFid,
        CSystemFidParameters &collectedParams,
        CMwNod *owner) const {
    CSystemFid *currentFid = sourceFid;
    CFastString packElemName;

    for (;;) {
        const CSystemPackDesc *packDesc = nullptr;
        unsigned long packElemIndex = 0ul;
        DoOneRemap(
                currentFid,
                packDesc,
                packElemName,
                packElemIndex,
                collectedParams);

        if (!packElemName.Empty()) {
            CSystemPackManager *packManager = CSystemEngine::s_PackManager;
            if (packManager != nullptr) {
                outFid = packManager->GetPackElem(
                        const_cast<CSystemPackDesc *>(packDesc),
                        packElemName,
                        packElemIndex,
                        currentFid,
                        owner);
            }
        }

        if (outFid == currentFid || packDesc != nullptr) {
            break;
        }
        sourceFid = currentFid;
        currentFid = outFid;
    }
}

void CSystemFidParameters::AddTemporaryFidRemap(
        CSystemFid *fid,
        CSystemPackDesc *packDesc,
        const CFastString *prefix,
        unsigned long id) {
    SParam_Fid param(fid, packDesc, *prefix, id);
    AddParam(param);
}

void CSystemFidParameters::AddTemporaryFidsRemap(
        CSystemFids *fids,
        CSystemPackDesc *packDesc,
        const CFastString *prefix,
        unsigned long id,
        int ensureTrailingBackslash) {
    SParam_Fids param(fids, packDesc, *prefix, id);
    if (ensureTrailingBackslash != 0 &&
        (param.prefix.Empty() || param.prefix.String().back() != '\\')) {
        param.prefix.Concat('\\');
    }
    AddParam(param);
}

void CSystemFidParameters::AddAllParamsFrom(
        const CSystemFidParameters &source) {
    for (const OwnedParameter &parameter : source.parameters) {
        AddParam(*parameter);
    }
}

int CSystemFidParameters::MergeForAudience(
        const CSystemFidParameters &source,
        const AssetAudience &audience) {
    inclusive = inclusive && source.inclusive;
    int changed = 0;
    for (const OwnedParameter &parameter : source.parameters) {
        const SParam &param = *parameter;
        if (!inclusive && param.IsDefault() != 0) {
            continue;
        }
        if (param.MatchesAudience(audience) == 0) {
            continue;
        }
        changed |= AddParam(param);
    }
    return changed;
}

int CSystemFidParameters::MergeForClassIds(
        const CSystemFidParameters &source,
        const CFastBuffer<CSystemFid *> &fids) {
    inclusive = inclusive && source.inclusive;
    int changed = 0;
    for (const OwnedParameter &parameter : source.parameters) {
        const SParam &param = *parameter;
        if (!inclusive && param.IsDefault() != 0) {
            continue;
        }
        if (param.MatchesAnyFids(fids) == 0) {
            continue;
        }
        changed |= AddParam(param);
    }
    return changed;
}

int CSystemFidParameters::MergeForFid(
        const CSystemFidParameters &source,
        CSystemFid *fid) {
    inclusive = inclusive && source.inclusive;
    int changed = 0;
    for (const OwnedParameter &parameter : source.parameters) {
        const SParam &param = *parameter;
        if (!inclusive && param.IsDefault() != 0) {
            continue;
        }
        if (fid != nullptr && param.MatchesFid(*fid) == 0) {
            continue;
        }
        changed |= AddParam(param);
    }
    return changed;
}

CSystemFidParameters::CSystemFidParameters(
        const CSystemFidParameters &other)
        : parameters(CloneParameters(other.parameters)),
          inclusive(other.inclusive) {}

CSystemFidParameters &CSystemFidParameters::operator=(
        const CSystemFidParameters &other) {
    if (this == &other) {
        return *this;
    }
    ParameterCollection replacement = CloneParameters(other.parameters);
    parameters = std::move(replacement);
    inclusive = other.inclusive;
    return *this;
}

void CSystemFidParameters::Pop(const CSystemFidParameters *params) {
    (void)params;
    auto &frames = CSystemFidParameterFrames();
    if (frames.empty()) {
        std::abort();
    }
    frames.pop_back();
}

void CSystemFidParameters::PopForTarget(CSystemFid *fid) {
    (void)fid;
    if (!CSystemFidParameterFrames().empty()) {
        Pop(nullptr);
    }
}

void CSystemFidParameters::StaticRelease(void) {
    CSystemFidParameterFrames().clear();
}

std::size_t CSystemFidParameters::ActiveScopeCount(void) noexcept {
    return CSystemFidParameterFrames().size();
}

const CSystemFidParameters &
CSystemFidParameters::GetCurrentParameters(void) {
    const auto &frames = CSystemFidParameterFrames();
    return frames.empty() ? Empty : *frames.back();
}

void CSystemFidParameters::Push(
        const CSystemFidParameters *params,
        int mergeWithPrevious) {
    auto &frames = CSystemFidParameterFrames();
    const CSystemFidParameters &previous =
            frames.empty() ? Empty : *frames.back();
    auto combined = std::make_unique<CSystemFidParameters>();

    if (mergeWithPrevious != 0) {
        *combined = previous;
        combined->AddAllParamsFrom(*params);
    } else {
        *combined = *params;
        combined->AddAllParamsFrom(previous);
    }
    combined->inclusive = false;
    combined->SimplifyForAudience(AnyAssetAudience());
    frames.emplace_back(std::move(combined));
}

void CSystemFidParameters::PushForAudience(
        CSystemFid *fid,
        const AssetAudience &audience) {
    (void)fid;
    auto &frames = CSystemFidParameterFrames();
    if (frames.empty()) {
        return;
    }

    const CSystemFidParameters &previous = *frames.back();
    auto combined = std::make_unique<CSystemFidParameters>();
    for (const OwnedParameter &parameter : previous.parameters) {
        const SParam &param = *parameter;
        if (param.MatchesAudience(audience) != 0) {
            combined->AddParam(param);
        }
    }
    frames.emplace_back(std::move(combined));
}

void CSystemFidParameters::PushForTarget(CSystemFid *fid) {
    auto &frames = CSystemFidParameterFrames();
    if (frames.empty()) {
        return;
    }

    const CSystemFidParameters &previous = *frames.back();
    auto combined = std::make_unique<CSystemFidParameters>();
    for (const OwnedParameter &parameter : previous.parameters) {
        const SParam &param = *parameter;
        if (fid == nullptr || param.MatchesFid(*fid) != 0) {
            combined->AddParam(param);
        }
    }
    frames.emplace_back(std::move(combined));
}

int CSystemFidParameters::RemappedLoadFromFid(
        CMwNod *&outNod,
        CSystemFid *fid,
        CMwNod *owner) {
    CSystemFid *sourceFid = fid;
    CSystemFid *loadFid = fid;
    const CSystemFidParameters &currentParameters =
            GetCurrentParameters();

    CSystemFidParameters collectedParams;
    currentParameters.Remap(
            sourceFid,
            loadFid,
            collectedParams,
            owner);

    const Scope parameterScope(loadFid);
    return loadFid != nullptr ? loadFid->LoadNode(outNod) : 0;
}

template <typename T>
int CSystemFidParameters::RemappedLoadFromFid(
        CMwNodRef<T> &outNod,
        CSystemFid *fid,
        CMwNod *owner) {
    static_assert(std::is_same_v<T, CMwNod>);

    CMwNod *loadedNod = nullptr;
    const int result = RemappedLoadFromFid(loadedNod, fid, owner);
    outNod.MwSetNod(loadedNod);
    return result;
}

template int CSystemFidParameters::RemappedLoadFromFid<CMwNod>(
        CMwNodRef<CMwNod> &outNod,
        CSystemFid *fid,
        CMwNod *owner);

int CSystemFidParameters::RemappedLoadFromFid_ParamFid(
        CMwNod *&outNod,
        CSystemFid *fid,
        CSystemFid *paramFid,
        CSystemFidFile *fidFile) {
    CSystemFidParameters parameters(Empty);

    CFastStringInt fullName;
    fidFile->GetFullName(fullName, 1, 0);
    CFastString prefix;
    fullName.GetUtf8OrAscii(prefix);

    const SParam_Fid remap(
            paramFid,
            nullptr,
            prefix,
            DefaultPackElemIndex);
    parameters.AddParam(remap);

    const Scope parameterScope(parameters, 1);
    const int loaded = RemappedLoadFromFid(outNod, fid, nullptr);
    return loaded != 0 && outNod != nullptr;
}

void CSystemFidParameters::DumpUsageStatistics(void) {}
