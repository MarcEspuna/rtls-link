#pragma once

#include <etl/span.h>
#include <etl/variant.h>

#include <SimpleKalmanFilter.h>

#include "trilat_generic.hpp"

#include "uwb_backend.hpp"

class UWBTag : public UWBBackend {
public:
    UWBTag(UWBFront& front, const bsp::UWBConfig& uwb_config, etl::span<const UWBAnchorParam> anchors);

    void Update() override;

    bool Start() override;
};