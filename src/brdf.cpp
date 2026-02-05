#include "brdf.h"
#include "utils.h"

using namespace math;

namespace {

	float DiffuseEnergyFactor(float roughness)
	{
		return math::lerp(1.0f, 1.0f / 1.51f, roughness);
	}

	Vector2 FresnelComponent(Vector2 LdotH)
	{
		return Vector2(std::powf(1.0f - LdotH.x(), 5), std::powf(1.0f - LdotH.y(), 5));
	}

	float FresnelComponent(float LdotH)
	{
		return std::powf(1.0f - LdotH, 5);
	}

	Vector3 DiffuseBurley(Vector3 albedo, float roughness, float NdotV, float NdotL, float LdotH)
	{
		const float energyBias = math::lerp(0.f, 0.5f, roughness);
		const float energyFactor = DiffuseEnergyFactor(roughness);

		const float FD90 = energyBias + 2.0f * LdotH * LdotH * roughness;
		const Vector2 FdVL = 1 + (FD90 - 1) * FresnelComponent(Vector2(NdotV, NdotL));

		return albedo * (FdVL.x() * FdVL.y()) * energyFactor * INV_PI;
	}


	float NDF(float NdotH, float GGXalpha)
	{
		// GGX / Trowbridge-Reitz
		// [Walter et al. 2007, "Microfacet models for refraction through rough surfaces"]
		const float denominator = ((GGXalpha * GGXalpha - 1.0f) * NdotH * NdotH + 1.0f);
		const float d = GGXalpha / std::max(EPS, denominator);
		return d * d * INV_PI;
	}

	float VF(float NdotL, float NdotV, float GGXalpha)
	{
		// Appoximation of joint Smith term for GGX
		// [Heitz 2014, "Understanding the Masking-Shadowing Function in Microfacet-Based BRDFs"]
		// Unreal Engine 4
		const float invAlpha = 1 - GGXalpha;
		const float lambdaV = NdotL * (NdotV * invAlpha + GGXalpha);
		const float lambdaL = NdotV * (NdotL * invAlpha + GGXalpha);

		// EPS prevent random sparkles due zero division and specular infinity values on mobile platforms
		return 0.5f / std::max(EPS, lambdaV + lambdaL);
	}

	Vector3 FresnelSchlick(Vector3 specColor, float LdotH)
	{
		// [Schlick 1994, "An Inexpensive BRDF Model for Physically-Based Rendering"]
		// values less 0.02 is incorrect and pretend to be specular occlusion
		return (specColor + (saturate(50.0f * specColor.y()) - specColor) * FresnelComponent(LdotH));
	}
}

Vector3 BRDF(const Vector3& inputAlbedo, float metallic, float roughness, const Vector3& L, const Vector3& H, const Vector3& N, const Vector3& V)
{
	auto specColor = math::lerp(Vector3(0.04f, 0.04f, 0.04f), inputAlbedo, metallic);
	auto albedo = math::lerp(inputAlbedo, Vector3(), metallic);
	const float NdotL = math::saturate(dot(N, L));

	roughness = std::max(roughness, 0.005f);

	// Avoid division by 0 in GGX formula in case NdotV == 0
	const float  NdotV = saturate(std::abs(dot(N, V)) + 1e-5f);
	const float NdotH = saturate(dot(N, H));
	const float LdotH = saturate(dot(L, H));

	const float GGXalpha = roughness * roughness;

	const Vector3 diffuse = DiffuseBurley(albedo, roughness, NdotV, NdotL, LdotH);
	const Vector3 specular = NDF(NdotH, GGXalpha) * VF(NdotL, NdotV, GGXalpha) * FresnelSchlick(specColor, LdotH);

	return diffuse + specular;
}
