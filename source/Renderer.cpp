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

			Vector4 rayDirection{ camera.cameraToWorld.TransformVector(cameraSpaceDir).ToVector4()};


			Ray viewRay{ camera.origin, rayDirection};

			ColorRGB finalColor{ };
			
			HitRecord closestHit{};
			
			pScene->GetClosestHit(viewRay, closestHit);


			if (closestHit.didHit)
			{
				finalColor = materials[closestHit.materialIndex]->Shade();

				for (size_t i = 0; i < pScene->GetLights().size(); i++)
				{

					Vector3 shadowDir = LightUtils::GetDirectionToLight(pScene->GetLights()[i], closestHit.origin).Normalized();

					Ray shadowRay{ closestHit.origin + 0.001 * shadowDir,  shadowDir};

					shadowRay.max = shadowDir.Normalize();
					shadowRay.min = 0.01f;


					if (pScene->DoesHit(shadowRay))
					{
						finalColor *= 0.5f;
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
