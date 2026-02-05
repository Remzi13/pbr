#pragma once

#include "vector.h"

Vector3 BRDF(const Vector3& inputAlbedo, float metallic, float roughness, const Vector3& L, const Vector3& H, const Vector3& N, const Vector3& V);