#pragma once
#include <cassert>
#include "Math.h"

namespace dae
{
	namespace BRDF
	{
		/**
		 * \param kd Diffuse Reflection Coefficient
		 * \param cd Diffuse Color
		 * \return Lambert Diffuse Color
		 */
		static ColorRGB Lambert(float kd, const ColorRGB& cd)
		{
			return (kd * cd) / static_cast<float>(M_PI);
		}

		static ColorRGB Lambert(const ColorRGB& kd, const ColorRGB& cd)
		{
			return (kd * cd) / static_cast<float>(M_PI);
		}

		/**
		 * \brief todo
		 * \param ks Specular Reflection Coefficient
		 * \param exp Phong Exponent
		 * \param l Incoming (incident) Light Direction
		 * \param v View Direction
		 * \param n Normal of the Surface
		 * \return Phong Specular Color
		 */
		static ColorRGB Phong(float ks, float exp, const Vector3& l, const Vector3& v, const Vector3& n)
		{
			ColorRGB phong = {};
			Vector3 reflect = Vector3::Reflect(l, n);
			float cosA = std::max(Vector3::Dot(reflect, v), 0.0f);
			if (cosA > 0.f)
			{
				phong +=  ColorRGB{ 1.f, 1.f, 1.f } * ks * powf(cosA, exp);
				return  phong;
			}
			return phong;
		}

		/**
		 * \brief BRDF Fresnel Function >> Schlick
		 * \ ( param h Normalized Halfvector between View and Light directions )
		 * 
		 * \param h is replaced with the surfaces normal param n
		 * 
		 * \param v Normalized View direction
		 * \param f0 Base reflectivity of a surface based on IOR (Indices Of Refrection), this is different for Dielectrics (Non-Metal) and Conductors (Metal)
		 * \return
		 */
		static ColorRGB FresnelFunction_Schlick(const Vector3& n, const Vector3& v, const ColorRGB& f0)
		{
			ColorRGB F = f0 + (ColorRGB{ 1.0f, 1.0f, 1.0f } - f0) * powf(1.f - Vector3::Dot(n , v), 5);

			return F;
		}

		/**
		 * \brief BRDF NormalDistribution >> Trowbridge-Reitz GGX (UE4 implemetation - squared(roughness))
		 * \param n Surface normal
		 * \param h Normalized half vector
		 * \param roughness Roughness of the material
		 * \return BRDF Normal Distribution Term using Trowbridge-Reitz GGX
		 */
		static float NormalDistribution_GGX(const Vector3& n, const Vector3& h, float roughness)
		{
			float a = roughness * roughness;
			float a2 = a * a;
			float NdotH = std::max(Vector3::Dot(n, h), 0.0f);
			float NdotH2 = NdotH * NdotH;

			float nom = a2;
			float denom = (NdotH2 * (a2 - 1.0f) + 1.0f);
			denom = static_cast<float>(M_PI) * denom * denom;

			return nom / denom;

		}


		/**
		 * \brief BRDF Geometry Function >> Schlick GGX (Direct Lighting + UE4 implementation - squared(roughness))
		 * \param n Normal of the surface
		 * \param v Normalized view direction
		 * \param k roughness  remapped based on direct lightning
		 * \return BRDF Geometry Term using SchlickGGX
		 */
		static float GeometryFunction_SchlickGGX(const Vector3& n, const Vector3& v, float k)
		{
			float G = Vector3::Dot(n , v) / (Vector3::Dot(n , v) * (1.f - k) + k);
			return G;
		}

		/**
		 * \brief BRDF Geometry Function >> Smith (Direct Lighting)
		 * \param n Normal of the surface
		 * \param v Normalized view direction
		 * \param l Normalized light direction
		 * \param roughness Roughness of the material
		 * \param k roughness  remapped based on direct lightning
		 * \return BRDF Geometry Term using Smith (> SchlickGGX(n,v,roughness) * SchlickGGX(n,l,roughness))
		 */
		static float GeometryFunction_Smith(const Vector3& n, const Vector3& v, const Vector3& l, float roughness)
		{
			float a = roughness * roughness;
			float k = ((a + 1.0f) * (a + 1.0f)) / 8;

			float Gsmith = GeometryFunction_SchlickGGX(n, -v, k) * GeometryFunction_SchlickGGX(n, l, k);
			return Gsmith;
		}

	}
}