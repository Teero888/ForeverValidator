#pragma once

#include "engine/rendering/plug_visual.h"
#include "engine/physics/geometry/plug_surface.h"
#include "engine/rendering/plug_tree.h"
#include "engine/core/engine_types.h"
#include "format/static_solid/static_solid_tree_assembler.h"
#include "format/static_solid/static_solid_archive_graph.h"
#include "format/static_solid/static_solid_generated_surfaces.h"
class StaticSolidArchiveLoadSession;

inline constexpr u32 StaticSolidHighestDecoratorQuality = 2u;

void ApplyStaticSolidDecoratorTreeQuality(
        CPlugTree *target,
        const CGameCtnReplayStaticSolidDecoratorTreeDeclaration &declaration,
        u32 quality);

class StaticSolidDecoratorAssembler {
public:
    void Free();
    int MaterializeDecoratorTrees(
            const CGameCtnReplayStaticSolidDecoratorTrees &decoratorTrees,
            const StaticSolidArchiveLoadSession &archive,
            StaticSolidTreeAssembler &archiveTrees,
            StaticSolidSurfaceAssembler &archiveSurfaces);
    void ApplyDecoratorTreeFlagsToWarpRoot(
            CPlugTree *root);

private:
    struct DecoratorSource {
        const CGameCtnReplayStaticSolidDecoratorTreeDeclaration
                *declaration = nullptr;
        CPlugTree *target = nullptr;
        CPlugVisual *visualProvider = nullptr;
        int usesGeneratedVisualSurface = 0;
        int usesSelectedDescriptorSurface = 0;
    };

    class DecoratorSourceSet {
    public:
        void Clear();
        int Add(const DecoratorSource &request,
                const CGameCtnReplayStaticSolidGeneratedSurfaceBudget
                        &requestBudget);
        const std::vector<DecoratorSource> &Requests() const;
        const CGameCtnReplayStaticSolidGeneratedSurfaceBudget
                &GeneratedSurfaceBudget() const;
        int HasGeneratedSurfaces() const;

    private:
        std::vector<DecoratorSource> requests;
        CGameCtnReplayStaticSolidGeneratedSurfaceBudget generatedSurfaceBudget;
    };

    CPlugTree *ResolveTree(CPlugTree *root, const CMwId &id) const {
        return root != nullptr ? root->GetPlugFromId(id) : nullptr;
    }
    void ApplyTreeProperties(
            CPlugTree *target,
            const CGameCtnReplayStaticSolidDecoratorTreeDeclaration &row);
    int AddVisualSurfaceBudget(
            CPlugVisual *provider,
            CGameCtnReplayStaticSolidGeneratedSurfaceBudget *budget)
            const {
        return generatedSurfaces.AddVisualSurfaceBudget(provider, budget);
    }
    int CollectSources(
            StaticSolidTreeAssembler &archiveTrees,
            DecoratorSourceSet *requests);
    int InstallDeclarations(
            const CGameCtnReplayStaticSolidDecoratorTrees &decoratorTrees);
    int ApplySelectedDescriptorSurface(
            const DecoratorSource &request,
            const StaticSolidArchiveLoadSession &archive,
            StaticSolidSurfaceAssembler &archiveSurfaces);
    int BuildGeneratedSurface(
            const DecoratorSource &request,
            CGameCtnReplayStaticSolidGeneratedSurfaceCursor *cursor);
    CPlugSurface *SurfaceForSelectedDescriptor(
            const StaticSolidArchiveLoadSession &archive,
            StaticSolidSurfaceAssembler &archiveSurfaces,
            const char *selectedDescriptorPath) const;

    StaticSolidGeneratedSurfaces generatedSurfaces;
    std::vector<CGameCtnReplayStaticSolidDecoratorTreeDeclaration>
            declarations;
};
