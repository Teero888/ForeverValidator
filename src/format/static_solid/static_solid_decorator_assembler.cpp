#include "format/static_solid/static_solid_decorator_assembler.h"
#include <stdint.h>
#include <vector>

#include "format/static_solid/static_scene_archive_loader.h"
#include "format/static_solid/static_solid_archive_graph.h"
#include "format/static_solid/static_solid_archive_node_kind.h"
#include <new>

void ApplyStaticSolidDecoratorTreeQuality(
        CPlugTree *target,
        const CGameCtnReplayStaticSolidDecoratorTreeDeclaration &declaration,
        u32 quality) {
    if (target == nullptr) {
        return;
    }

    const bool shown = declaration.IsShownAtQuality(quality) != 0;
    target->ApplyDecorationFlags(
            shown && declaration.IsVisibleAtQuality(quality),
            shown && declaration.CastsShadowsAtQuality(quality),
            shown && declaration.HasCollisionAtQuality(quality),
            declaration.AppliesVisibleToChildren(),
            declaration.AppliesCasterToChildren());
}

void StaticSolidDecoratorAssembler::Free() {
    generatedSurfaces.Free();
    declarations.clear();
}

void StaticSolidDecoratorAssembler::ApplyTreeProperties(
        CPlugTree *target,
        const CGameCtnReplayStaticSolidDecoratorTreeDeclaration
                &declaration) {
    ApplyStaticSolidDecoratorTreeQuality(
            target, declaration, StaticSolidHighestDecoratorQuality);
}

CPlugSurface *StaticSolidDecoratorAssembler::
        SurfaceForSelectedDescriptor(
                const StaticSolidArchiveLoadSession &archive,
                StaticSolidSurfaceAssembler &archiveSurfaces,
                const char *selectedDescriptorPath) const {
    const StaticSolidArchiveId payload =
            archive.SelectPayloadForDescriptor(selectedDescriptorPath);
    if (!payload.IsValid()) {
        return nullptr;
    }
    return archiveSurfaces.SurfaceForPayload(payload);
}

void StaticSolidDecoratorAssembler::
        DecoratorSourceSet::Clear() {
    requests.clear();
    generatedSurfaceBudget.Clear();
}

int StaticSolidDecoratorAssembler::
        DecoratorSourceSet::Add(
                const DecoratorSource &request,
                const CGameCtnReplayStaticSolidGeneratedSurfaceBudget
                        &requestBudget) {
    try {
        requests.push_back(request);
    } catch (const std::bad_alloc &) {
        return 0;
    }
    generatedSurfaceBudget.Add(requestBudget);
    return 1;
}

const std::vector<StaticSolidDecoratorAssembler::
                          DecoratorSource> &
StaticSolidDecoratorAssembler::
        DecoratorSourceSet::Requests() const {
    return requests;
}

const CGameCtnReplayStaticSolidGeneratedSurfaceBudget &
StaticSolidDecoratorAssembler::
        DecoratorSourceSet::GeneratedSurfaceBudget() const {
    return generatedSurfaceBudget;
}

int StaticSolidDecoratorAssembler::
        DecoratorSourceSet::HasGeneratedSurfaces() const {
    return !generatedSurfaceBudget.Empty();
}

int StaticSolidDecoratorAssembler::CollectSources(
        StaticSolidTreeAssembler &archiveTrees,
        DecoratorSourceSet *requests) {
    if (requests == nullptr) {
        return 0;
    }
    requests->Clear();

    const u32 quality = StaticSolidHighestDecoratorQuality;
    int ok = 1;
    for (const CGameCtnReplayStaticSolidDecoratorTreeDeclaration
                 &declaration : declarations) {
        DecoratorSource request;
        request.declaration = &declaration;
        request.usesGeneratedVisualSurface =
                declaration.UsesVisualSurfaceAtQuality(quality);
        request.usesSelectedDescriptorSurface =
                declaration.HasSelectedDescriptorPath() &&
                declaration.IsShownAtQuality(quality);

        CPlugTree *root =
                archiveTrees.CollisionRootForPayload(
                        declaration.Decorator().Payload());
        request.target = ResolveTree(root, declaration.TreeId());
        request.visualProvider = request.target != nullptr
                ? request.target->Visual()
                : nullptr;

        CGameCtnReplayStaticSolidGeneratedSurfaceBudget requestBudget;
        if (request.usesGeneratedVisualSurface) {
            requestBudget.ReserveCandidateSurface();
            AddVisualSurfaceBudget(request.visualProvider, &requestBudget);
        }

        if (!requests->Add(request, requestBudget)) {
            ok = 0;
            break;
        }
    }
    return ok;
}

int StaticSolidDecoratorAssembler::InstallDeclarations(
        const CGameCtnReplayStaticSolidDecoratorTrees &decoratorTrees) {
    declarations.clear();
    int ok = 1;
    decoratorTrees.ForEachDeclaration([&](
            const CGameCtnReplayStaticSolidDecoratorTreeDeclaration
                    &declaration) {
        try {
            declarations.push_back(declaration);
        } catch (const std::bad_alloc &) {
            ok = 0;
            return 0;
        }
        return 1;
    });
    return ok;
}

int StaticSolidDecoratorAssembler::
        ApplySelectedDescriptorSurface(
                const DecoratorSource &request,
                const StaticSolidArchiveLoadSession &archive,
                StaticSolidSurfaceAssembler &archiveSurfaces) {
    if (request.declaration == nullptr ||
        !request.usesSelectedDescriptorSurface) {
        return 1;
    }
    CPlugSurface *surface =
            SurfaceForSelectedDescriptor(archive,
                                         archiveSurfaces,
                                         request.declaration
                                                 ->SelectedDescriptorPath());
    if (request.target == nullptr || surface == nullptr) {
        return 1;
    }
    request.target->SetSurface(surface);
    return 1;
}

int StaticSolidDecoratorAssembler::BuildGeneratedSurface(
        const DecoratorSource &request,
        CGameCtnReplayStaticSolidGeneratedSurfaceCursor *cursor) {
    if (request.declaration == nullptr || !request.usesGeneratedVisualSurface) {
        return 1;
    }
    if (request.target == nullptr) {
        return 1;
    }
    if (request.visualProvider == nullptr) {
        return 1;
    }
    const CGameCtnReplayStaticSolidGeneratedSurfaceVisual visual(
            request.visualProvider);
    if (!visual.CanBuildMesh()) {
        return 1;
    }
    if (cursor == nullptr ||
        !generatedSurfaces.CreateSurfaceFromVisual(request.visualProvider,
                                                   cursor)) {
        return 1;
    }
    request.target->SetSurface(
            generatedSurfaces.SurfaceAt(cursor->SurfaceIndex()));
    request.target->EnableCollisionOnAncestors();
    return 1;
}

void StaticSolidDecoratorAssembler::
        ApplyDecoratorTreeFlagsToWarpRoot(
        CPlugTree *root) {
    if (root == nullptr || declarations.empty()) {
        return;
    }

    /*
     * Decorator declarations target named descendants of the Warp tree. Apply
     * each declaration to the descendant selected by its archived tree id.
     */
    for (const CGameCtnReplayStaticSolidDecoratorTreeDeclaration
                 &declaration : declarations) {
        ApplyTreeProperties(ResolveTree(root, declaration.TreeId()), declaration);
    }
}

int StaticSolidDecoratorAssembler::MaterializeDecoratorTrees(
        const CGameCtnReplayStaticSolidDecoratorTrees &decoratorTrees,
        const StaticSolidArchiveLoadSession &archive,
        StaticSolidTreeAssembler &archiveTrees,
        StaticSolidSurfaceAssembler &archiveSurfaces) {
    if (decoratorTrees.Empty()) {
        return 1;
    }
    if (!InstallDeclarations(decoratorTrees)) {
        return 0;
    }

    DecoratorSourceSet requests;
    if (!CollectSources(archiveTrees, &requests)) {
        return 0;
    }

    for (const DecoratorSource &request : requests.Requests()) {
        ApplySelectedDescriptorSurface(request,
                                       archive,
                                       archiveSurfaces);
    }

    for (const DecoratorSource &request : requests.Requests()) {
        if (request.declaration != nullptr) {
            ApplyTreeProperties(request.target, *request.declaration);
        }
    }

    const CGameCtnReplayStaticSolidGeneratedSurfaceBudget &generatedBudget =
            requests.GeneratedSurfaceBudget();
    if (!generatedSurfaces.Allocate(generatedBudget)) {
        return 0;
    }
    if (!requests.HasGeneratedSurfaces()) {
        return 1;
    }

    CGameCtnReplayStaticSolidGeneratedSurfaceCursor cursor;
    for (const DecoratorSource &request : requests.Requests()) {
        BuildGeneratedSurface(request, &cursor);
    }

    return 1;
}
