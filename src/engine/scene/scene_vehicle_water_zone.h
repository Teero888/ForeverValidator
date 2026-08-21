#pragma once

#include "engine/core/engine_types.h"
#include "engine/core/gm_map2.h"
#include "engine/core/gm_types.h"
#include "engine/game/water_occupancy.h"
class CSceneVehicleWaterZone {
public:
    CSceneVehicleWaterZone(void);
    void Disable(void);
    void Configure(const WaterOccupancyGrid &occupancy,
                   float surfaceHeight,
                   float secondaryCullHeight);
    bool IsEnabled(void) const noexcept { return enabled; }
    bool AcceptsRegion(const GmVec2 &sample,
                       float lowerY,
                       float upperY) const;
    float SurfaceHeight(void) const { return surfaceHeight; }

private:
    bool enabled = false;
    GmMap2<unsigned char> waterMap;
    float surfaceHeight = 0.0f;
    float secondaryCullHeight = 0.0f;
};
