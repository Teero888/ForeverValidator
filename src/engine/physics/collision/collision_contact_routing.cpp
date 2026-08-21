// Contact-buffer selection, actor binding, and surface dispatch.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "engine/core/cmw_nod.h"
#include "engine/physics/geometry/geometry_helpers.h"
#include "engine/physics/collision/gm_collision_buffer.h"
#include "engine/physics/collision/hms_collision.h"
#include "engine/physics/collision/hms_collision_manager.h"
#include "engine/physics/dynamics/hms_corpus.h"
#include "engine/physics/dynamics/hms_dyna.h"
#include "engine/physics/dynamics/hms_item.h"
#include "engine/physics/world/hms_zone.h"
#include "engine/physics/geometry/physics_tolerances.h"
#include "engine/scene/plug_solid.h"
#include "engine/physics/geometry/plug_surface.h"
#include "engine/rendering/plug_tree.h"

SHmsSphereBufferContact *
CHmsCollisionManager::SZone::EnsureTreeSphereContactSlow(
        CPlugTree *tree,
        TreeSphereContactCacheEntry &cached) {
    for (TreeSphereContact &owned : ownedSphereContacts) {
        if (owned.tree == tree) {
            cached = {tree, owned.contact.get()};
            return cached.contact;
        }
    }
    TreeSphereContact owned;
    owned.tree = tree;
    owned.contact = std::make_unique<SHmsSphereBufferContact>();
    SHmsSphereBufferContact *contact = owned.contact.get();
    ownedSphereContacts.push_back(std::move(owned));
    cached = {tree, contact};
    return contact;
}

void CHmsCollisionManager::SZone::AddSphereContactOnce(SHmsSphereBufferContact *sphereContact) {
    if (!sphereContact->queuedForMerge) {
        sphereContact->queuedForMerge = true;
        sphereBufferContacts.push_back(sphereContact);
    }
}

void CHmsCollisionManager::SZone::MergeQueuedSphereContacts(
        CHmsCollisionBuffer &collisionBuffer) {
    for (SHmsSphereBufferContact *contact : sphereBufferContacts) {
        contact->MergeAndAddToCollisions(collisionBuffer);
    }
    sphereBufferContacts.clear();
}

void CHmsCollisionManager::SZone::BeginReplayStaticCollisionPass(
        CHmsCollisionBuffer *collisionBuffer,
        CHmsCorpus *sourceCorpus) {
    activeCollisionBuffer = collisionBuffer;
    activeCorpusA = sourceCorpus;
}

void CHmsCollisionManager::SZone::SelectReplayStaticCollisionTarget(
        const SGroup::SAgainstGroup &against) {
    activeCollisionGroupPair = against.CollisionGroupPair();
    activeStaticTargetGroup = against.TargetGroup();
}

CHmsCollisionBuffer *CHmsCollisionManager::SZone::ChooseCollisionOutputBuffer(
    CPlugTree *treeA,
    CPlugSurface *surfaceA,
    SHmsSphereBufferContact **sphereContactOut) {
    *sphereContactOut = 0;
    if (surfaceA->UsesSphereContactBuffer()) {
        *sphereContactOut = EnsureTreeSphereContact(treeA);
        return *sphereContactOut;
    }
    return activeCollisionBuffer;
}

void CHmsCollisionManager::SZone::TagNewCollisions(CHmsCollisionBuffer *buffer,
                                                   u32 firstNew,
                                                   CPlugTree *treeA,
                                                   CPlugTree *treeB) {
    u32 count = buffer->PhysicalCollisionCount();
    for (u32 collisionIndex = firstNew; collisionIndex < count; collisionIndex++) {
        SHmsPhysicalCollision *collision = buffer->PhysicalCollisionAtOrNull(collisionIndex);
        if (collision == nullptr) {
            continue;
        }
        collision->BindCollisionActors(
                activeCorpusA,
                treeA,
                activeCorpusB,
                treeB,
                activeCollisionGroupPair);
    }
}

void CHmsCollisionManager::SZone::TagNewStaticCollisions(
    CHmsCollisionBuffer *buffer,
    u32 firstNew,
    CPlugTree *movingTree,
    const CHmsCollisionManagerSColOctreeCell *record) {
    const SColOctreeCell::StaticSurface &surface = record->SurfaceData();
    u32 count = buffer->PhysicalCollisionCount();
    for (u32 collisionIndex = firstNew; collisionIndex < count; collisionIndex++) {
        SHmsPhysicalCollision *collision = buffer->PhysicalCollisionAtOrNull(collisionIndex);
        if (collision == nullptr) {
            continue;
        }
        collision->BindCollisionActors(
                activeCorpusA,
                movingTree,
                surface.corpus,
                surface.tree,
                activeCollisionGroupPair);
    }
}

int CHmsCollisionManager::SZone::ComputeSurfaceCollisionAndTag(
    CPlugTree *treeA,
    const GmIso4 *isoA,
    CPlugSurface *surfaceA,
    CPlugTree *treeB,
    const GmIso4 *isoB,
    CPlugSurface *surfaceB) {
    SHmsSphereBufferContact *sphereContact = 0;
    CHmsCollisionBuffer *buffer = ChooseCollisionOutputBuffer(treeA, surfaceA, &sphereContact);
    u32 firstNew = buffer->PhysicalCollisionCount();
    SPlugSurfaceLocatedPair surfacePair = {
        *surfaceA,
        *isoA,
        *surfaceB,
        *isoB,
    };

    if (!CPlugSurface::ComputeCollision(surfacePair, *buffer)) {
        return 0;
    }
    if (sphereContact != 0) {
        AddSphereContactOnce(sphereContact);
    }
    TagNewCollisions(buffer, firstNew, treeA, treeB);
    return 1;
}
