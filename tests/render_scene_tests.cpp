#include <forevervalidator/experimental/physics_sandbox.h>

#include "engine/game/game_ctn_block_info.h"
#include "engine/rendering/plug_tree.h"
#include "engine/scene/static_scene_model.h"
#include "format/static_solid/static_solid_decorator_assembler.h"
#include "format/static_solid/static_solid_geometry_decoder.h"
#include "simulation/replay/replay_scene_surface_resolution.h"
#include "simulation/runtime/replay_simulation_session.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

bool Check(bool condition, const char *message) {
    if (!condition) {
        std::cerr << message << '\n';
    }
    return condition;
}

class TestBlockInfoAssetRegistry : public BlockInfoAssetRegistry {
public:
    static BlockInfoAssetHandle Handle(u32 index) {
        return HandleForStorageIndex(index);
    }
};

class JunctionResolverRepository final : public CatalogAssetRepository {
public:
    JunctionResolverRepository(
            BlockInfoAssetHandle sourceAsset,
            CGameCtnBlockInfoClip &sourceClip)
            : sourceAsset_(sourceAsset), sourceClip_(&sourceClip) {}

    const BlockInfoCatalog *Catalog() override { return nullptr; }

    CGameCtnBlockInfo *BlockInfo(BlockInfoAssetHandle asset) override {
        return asset == sourceAsset_ ? sourceClip_.Get() : nullptr;
    }

    CSceneMobil *Mobil(
            BlockInfoAssetHandle,
            bool,
            u32) override {
        return nullptr;
    }

    std::optional<std::string> FirstGroundSurface(
            BlockInfoAssetHandle) override {
        return std::nullopt;
    }

    std::optional<CatalogCollectionDefinition> Collection(
            std::string_view) override {
        return std::nullopt;
    }

    std::optional<CatalogDecorationSizeDefinition> DecorationSize(
            const CGameCtnReplayMapInput &) override {
        return std::nullopt;
    }

    bool HasSurfaceReplacement(
            std::string_view,
            std::string_view,
            std::string_view) override {
        return false;
    }

private:
    BlockInfoAssetHandle sourceAsset_;
    CMwNodRef<CGameCtnBlockInfoClip> sourceClip_;
};

void AppendFloat(std::vector<std::uint8_t> *bytes, float value) {
    const std::size_t offset = bytes->size();
    bytes->resize(offset + sizeof(value));
    std::memcpy(bytes->data() + offset, &value, sizeof(value));
}

bool NearlyEqual(float lhs, float rhs) {
    return std::fabs(lhs - rhs) < 0.0001f;
}

bool TestUvDecoding() {
    std::vector<std::uint8_t> uv2Bytes;
    for (float value : {0.25f, 0.75f, -1.0f, 2.0f}) {
        AppendFloat(&uv2Bytes, value);
    }
    GxTexCoordSet uv2;
    bool okay = Check(
            DecodeStaticSolidTexCoordStream(
                    uv2Bytes.data(), 2u, 2u, 8u, &uv2),
            "2D UV stream was rejected");
    const GxTexCoord4 first = uv2.Coordinate4At(0u);
    const GxTexCoord4 second = uv2.Coordinate4At(1u);
    okay &= Check(
            uv2.Dimension() == GxTexCoordDimension::Two &&
                    uv2.Count() == 2u &&
                    NearlyEqual(first.u, 0.25f) &&
                    NearlyEqual(first.v, 0.75f) &&
                    NearlyEqual(second.u, -1.0f) &&
                    NearlyEqual(second.v, 2.0f),
            "2D UV values were not preserved");

    std::vector<std::uint8_t> uv3Bytes;
    for (float value : {-2.5f, 0.125f, 7.75f}) {
        AppendFloat(&uv3Bytes, value);
    }
    GxTexCoordSet uv3;
    okay &= Check(
            DecodeStaticSolidTexCoordStream(
                    uv3Bytes.data(), 1u, 3u, 12u, &uv3),
            "3D UV stream was rejected");
    const GxTexCoord4 threeDimensional = uv3.Coordinate4At(0u);
    okay &= Check(
            uv3.Dimension() == GxTexCoordDimension::Three &&
                    uv3.Count() == 1u &&
                    NearlyEqual(threeDimensional.u, -2.5f) &&
                    NearlyEqual(threeDimensional.v, 0.125f) &&
                    NearlyEqual(threeDimensional.w, 7.75f) &&
                    NearlyEqual(threeDimensional.q, 1.0f),
            "3D UV values were not preserved");

    std::vector<std::uint8_t> uv4Bytes;
    for (float value : {1.0f, 2.0f, 3.0f, 4.0f}) {
        AppendFloat(&uv4Bytes, value);
    }
    GxTexCoordSet uv4;
    okay &= Check(
            DecodeStaticSolidTexCoordStream(
                    uv4Bytes.data(), 1u, 4u, 16u, &uv4),
            "4D UV stream was rejected");
    const GxTexCoord4 expanded = uv4.Coordinate4At(0u);
    okay &= Check(
            uv4.Dimension() == GxTexCoordDimension::Four &&
                    NearlyEqual(expanded.u, 1.0f) &&
                    NearlyEqual(expanded.v, 2.0f) &&
                    NearlyEqual(expanded.w, 3.0f) &&
                    NearlyEqual(expanded.q, 4.0f),
            "4D UV values were not preserved");
    okay &= Check(
            !DecodeStaticSolidTexCoordStream(
                    uv2Bytes.data(), 2u, 2u, 12u, &uv2),
            "invalid UV stride was accepted");
    return okay;
}

bool TestTransformComposition() {
    GmIso4 parent;
    parent.SetIdentity();
    parent.SetTranslation({10.0f, 20.0f, 30.0f});
    GmIso4 local;
    local.SetIdentity();
    local.SetTranslation({1.0f, 2.0f, 3.0f});

    CPlugTree tree;
    tree.SetUseLocation(1);
    tree.SetLocation(local);
    GmIso4 world;
    tree.ComposeCollisionIso(parent, world);
    return Check(
            NearlyEqual(world.translation.x, 11.0f) &&
                    NearlyEqual(world.translation.y, 22.0f) &&
                    NearlyEqual(world.translation.z, 33.0f),
            "tree local transform was not composed with its parent");
}

bool TestHighestQualityVisualSelection() {
    CPlugTreeVisualMip mip;
    mip.AddOwnedLevel(std::make_unique<CPlugTree>(), 100.0f);
    mip.AddOwnedLevel(
            std::make_unique<CPlugTree>(),
            std::numeric_limits<float>::max());
    mip.SetQuality(100u);

    bool okay = Check(
            mip.GetChildToRenderMip() == mip.LevelTree(1u),
            "highest visual MIP quality did not select the last level");
    okay &= Check(
            PhysicsSandboxRenderLodLevelForVisualMip(0u, 1u, 2u) == 0u &&
                    PhysicsSandboxRenderLodLevelForVisualMip(0u, 0u, 2u) ==
                            1u,
            "visual MIP levels were not ranked from highest quality");
    okay &= Check(
            PhysicsSandboxRenderLodLevelForVisualMip(1u, 1u, 2u) == 1u,
            "nested visual MIP reset a lower-detail ancestor to LOD zero");

    CGameCtnReplayStaticSolidDecoratorTreeDeclaration lowOnly;
    lowOnly.SetVisibilityConditions(1u, 6u, 1u, 6u, 1u, 6u);
    lowOnly.SetCollisionCondition(6u);
    CPlugTree excluded;
    excluded.SetIsVisible(1);
    excluded.SetShadowCaster(true);
    excluded.SetCollisionEnabled(true);
    ApplyStaticSolidDecoratorTreeQuality(
            &excluded, lowOnly, StaticSolidHighestDecoratorQuality);
    okay &= Check(
            !excluded.IsVisible() && !excluded.IsShadowCaster() &&
                    !excluded.State().collisionEnabled,
            "lower-quality decorator tree remained active at highest quality");

    CGameCtnReplayStaticSolidDecoratorTreeDeclaration highOnly;
    highOnly.SetVisibilityConditions(5u, 5u, 0u, 5u, 0u, 5u);
    highOnly.SetCollisionCondition(5u);
    CPlugTree included;
    included.SetIsVisible(0);
    included.SetShadowCaster(false);
    included.SetCollisionEnabled(false);
    ApplyStaticSolidDecoratorTreeQuality(
            &included, highOnly, StaticSolidHighestDecoratorQuality);
    okay &= Check(
            included.IsVisible() && included.IsShadowCaster() &&
                    included.State().collisionEnabled,
            "highest-quality decorator tree was not activated");
    return okay;
}

bool TestProvenanceAndImmutableScene() {
    GmIso4 identity;
    identity.SetIdentity();
    StaticSceneModel model(
            StaticSolidPrototype{},
            identity,
            StaticScenePurpose::SubMobil);
    StaticSceneProvenance provenance;
    provenance.blockName = "StadiumRoadMain";
    provenance.collection = "Stadium";
    provenance.descriptorPath = "GameData/StadiumRoad.Block.Gbx";
    provenance.sceneObjectId = "mobil-3";
    provenance.placementIdentity = 42u;
    provenance.blockInstanceId = 7u;
    provenance.variant = 2u;
    provenance.componentIndex = 3u;
    provenance.authored = true;
    model.SetProvenance(provenance);

    StaticSceneModelCollection models;
    bool okay = Check(models.Add(std::move(model)),
                      "scene model could not be stored");
    const StaticSceneModel &stored = models.Models().front();
    okay &= Check(
            stored.Purpose() == StaticScenePurpose::SubMobil &&
                    stored.Provenance().blockName == "StadiumRoadMain" &&
                    stored.Provenance().placementIdentity == 42u &&
                    stored.Provenance().blockInstanceId == 7u &&
                    stored.Provenance().variant == 2u &&
                    stored.Provenance().componentIndex == 3u &&
                    stored.Provenance().authored,
            "authored provenance changed after scene-model storage");

    using Scene =
            forevervalidator::experimental::PhysicsSandboxRenderScene;
    using Handle =
            forevervalidator::experimental::
                    PhysicsSandboxRenderSceneHandle;
    static_assert(std::is_same_v<Handle, std::shared_ptr<const Scene>>);
    const Handle scene = std::make_shared<const Scene>();
    okay &= Check(scene->meshes.empty() && scene->instances.empty(),
                  "immutable render-scene handle was not readable");
    return okay;
}

bool TestGenericBackgroundLayerClassification() {
    using forevervalidator::experimental::PhysicsSandboxRenderInstance;
    using forevervalidator::experimental::PhysicsSandboxRenderLayer;
    using forevervalidator::experimental::PhysicsSandboxRenderMesh;
    using forevervalidator::experimental::PhysicsSandboxRenderScene;
    using forevervalidator::experimental::PhysicsSandboxScenePurpose;

    const auto mesh = [](forevervalidator::Vector3 minimum,
                         forevervalidator::Vector3 maximum) {
        PhysicsSandboxRenderMesh result;
        result.boundsMin = minimum;
        result.boundsMax = maximum;
        return result;
    };
    PhysicsSandboxRenderScene scene;
    scene.meshes.push_back(mesh(
            {-10.0f, 0.0f, -10.0f}, {10.0f, 10.0f, 10.0f}));
    scene.meshes.push_back(mesh(
            {-100.0f, -100.0f, -100.0f}, {100.0f, 0.0f, 100.0f}));
    scene.meshes.push_back(mesh(
            {-100.0f, 0.0f, -100.0f}, {100.0f, 100.0f, 100.0f}));
    scene.meshes.push_back(mesh(
            {-20.0f, -5.0f, -20.0f}, {20.0f, 20.0f, 20.0f}));

    PhysicsSandboxRenderInstance foreground;
    foreground.meshIndex = 0u;
    foreground.purpose = PhysicsSandboxScenePurpose::PlacedBlock;
    scene.instances.push_back(foreground);

    PhysicsSandboxRenderInstance lowerBackground;
    lowerBackground.meshIndex = 1u;
    lowerBackground.purpose = PhysicsSandboxScenePurpose::Environment;
    lowerBackground.provenance.descriptorPath = "Shared/Backdrop";
    scene.instances.push_back(lowerBackground);
    PhysicsSandboxRenderInstance upperBackground = lowerBackground;
    upperBackground.meshIndex = 2u;
    scene.instances.push_back(upperBackground);

    PhysicsSandboxRenderInstance nearbyEnvironment;
    nearbyEnvironment.meshIndex = 3u;
    nearbyEnvironment.purpose = PhysicsSandboxScenePurpose::Environment;
    nearbyEnvironment.provenance.descriptorPath = "Shared/Scenery";
    scene.instances.push_back(nearbyEnvironment);

    ClassifyPhysicsSandboxRenderLayers(scene);
    return Check(
            scene.instances[0].renderLayer ==
                            PhysicsSandboxRenderLayer::World &&
                    scene.instances[1].renderLayer ==
                            PhysicsSandboxRenderLayer::Background &&
                    scene.instances[2].renderLayer ==
                            PhysicsSandboxRenderLayer::Background &&
                    !scene.instances[1].castsShadows &&
                    !scene.instances[2].castsShadows &&
                    scene.instances[3].renderLayer ==
                            PhysicsSandboxRenderLayer::World,
            "generic enclosing backdrop was not separated from world geometry");
}

bool TestClipJunctionSourceResolution() {
    const BlockInfoAssetHandle sourceAsset =
            TestBlockInfoAssetRegistry::Handle(7u);
    CMwNodRef<CGameCtnBlockInfoClip> unresolved =
            MakeMwNod<CGameCtnBlockInfoClip>();
    CMwNodRef<CGameCtnBlockInfoClip> resolved =
            MakeMwNod<CGameCtnBlockInfoClip>();
    unresolved->SetSourceAsset(sourceAsset);
    resolved->SetSourceAsset(sourceAsset);
    resolved->ResetMobilVariants(CGameCtnBlockInfo::GroundMobilFamily, 1u);
    resolved->AddMobil(
            CGameCtnBlockInfo::GroundMobilFamily,
            0u,
            new CSceneMobil());

    CGameCtnBlockInfo owner;
    auto unit = std::make_unique<CGameCtnBlockUnitInfo>();
    unit->InitializeUnitFields({0u, 0u, 0u}, 0u, 0u, &owner);
    unit->SetJunction(ECardinalDir::West, unresolved.Get());
    CGameCtnBlockUnitInfo *installedUnit = unit.get();
    owner.AddBlockUnitInfo(true, std::move(unit));

    JunctionResolverRepository repository(sourceAsset, *resolved);
    ReplaySceneAssetResolver resolver;
    resolver.assets = &repository;
    resolver.ResolveJunctionSources(owner);

    CGameCtnBlockInfoClip *junction =
            installedUnit->JunctionAt(ECardinalDir::West);
    return Check(
            junction == resolved.Get() &&
                    junction->GetMobil(
                            CGameCtnBlockInfo::GroundMobilFamily,
                            0u,
                            0u) != nullptr,
            "deferred clip junction kept its empty placeholder");
}

}  // namespace

int main() {
    bool okay = TestUvDecoding();
    okay &= TestTransformComposition();
    okay &= TestHighestQualityVisualSelection();
    okay &= TestProvenanceAndImmutableScene();
    okay &= TestGenericBackgroundLayerClassification();
    okay &= TestClipJunctionSourceResolution();
    return okay ? 0 : 1;
}
