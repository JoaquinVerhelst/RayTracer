//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

#include <iostream>

using namespace dae;

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();


	//aspect ratio
	float ar = static_cast<float>(m_Width) / static_cast<float>(m_Height);
	float fov = tan(camera.fovAngle / 2.f);
	
	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{
			//float gradient = px / static_cast<float>(m_Width);
			//gradient += py / static_cast<float>(m_Width);
			//gradient /= 2.0f;


			float x = (((2 * (px + 0.5f)) / m_Width) - 1) * ar * fov;
			float y = (1 - ((2 * (py + 0.5f)) / m_Height)) * fov;

			Vector3 cameraSpaceDir{ x, y, 1 };
			cameraSpaceDir.ToVector4();

			camera.cameraToWorld = camera.CalculateCameraToWorld();

			Vector4 rayDirection{ camera.cameraToWorld.TransformVector(cameraSpaceDir).Normalized().ToVector4() };


			Ray viewRay{ camera.origin, rayDirection};

			ColorRGB finalColor{ };
			
			HitRecord closestHit{};
			
			pScene->GetClosestHit(viewRay, closestHit);


			if (closestHit.didHit)
			{
				Ray shadowRay{};
				shadowRay.min = 0.01f;

				Vector3 shadowDir{};

				for (size_t i = 0; i < pScene->GetLights().size(); i++)
				{
					shadowDir = LightUtils::GetDirectionToLight(pScene->GetLights()[i], closestHit.origin);
					shadowRay.max = shadowDir.Normalize();
					shadowRay.origin = closestHit.origin + shadowRay.min * shadowDir;
					shadowRay.direction = shadowDir;

					Vector3 v = closestHit.origin - viewRay.direction;

					if (Vector3::Dot(closestHit.normal, shadowDir) < 0 )
						continue;
					
					if (pScene->DoesHit(shadowRay) && m_ShadowsEnabled)
						continue;
					

					ColorRGB E = LightUtils::GetRadiance(pScene->GetLights()[i], closestHit.origin);

					ColorRGB BRDFrgb =  materials[closestHit.materialIndex]->Shade(closestHit, shadowDir.Normalized(), viewRay.direction.Normalized());

					switch (m_CurrentLightingMode)
					{
					case LightingMode::ObservedArea:

						finalColor += ColorRGB{ 1.f, 1.f, 1.f } * Vector3::Dot(closestHit.normal, shadowDir);
						break;
					case LightingMode::Radiance:

						finalColor += E;
						break;
					case LightingMode::BRDF:
						finalColor += BRDFrgb;
						break;
					case LightingMode::Combined:
						finalColor += E * BRDFrgb * (Vector3::Dot(closestHit.normal, shadowDir));
						break;
					}

				}
			}

			//Update Color in Buffer
			finalColor.MaxToOne();

			m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));

			//camera

		}
	}

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode()
{
	switch (m_CurrentLightingMode)
	{
	case LightingMode::ObservedArea:
		m_CurrentLightingMode = LightingMode::Radiance;
		break;
	case LightingMode::Radiance:
		m_CurrentLightingMode = LightingMode::BRDF;
		break;
	case LightingMode::BRDF:
		m_CurrentLightingMode = LightingMode::Combined;
		break;
	case LightingMode::Combined:
		m_CurrentLightingMode = LightingMode::ObservedArea;
		break;
	default:
		break;
	};


}
