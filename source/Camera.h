#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"


#include <iostream>



namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle):
			origin{_origin},
			fovAngle{ _fovAngle }
		{
			SetFovAngle(fovAngle);
		}


		Vector3 origin{};
		float fovAngle{90.f};

		Vector3 forward{Vector3::UnitZ};
		Vector3 up{Vector3::UnitY};
		Vector3 right{Vector3::UnitX};

		float totalPitch{0.f};
		float totalYaw{0.f};

		Matrix cameraToWorld{};

		float movementSpeed{4.f};
		float rotationSpeed{10.f * TO_RADIANS};

		bool isShiftPressed = false;


		Matrix CalculateCameraToWorld()
		{
			right = Vector3::Cross(Vector3::UnitY, forward).Normalized();
			up = Vector3::Cross(forward, right).Normalized();


			cameraToWorld = Matrix{ right, up, forward, origin };

			return cameraToWorld;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();


			float currentMovementSpeed{ movementSpeed };

			//Keyboard Input

			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

			if (pKeyboardState[SDL_SCANCODE_LSHIFT] == 1)
			{
				currentMovementSpeed *= 4;
			}


			if (pKeyboardState[SDL_SCANCODE_W] == 1)
				origin += movementSpeed * forward * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_S] == 1)
				origin -= movementSpeed * forward * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_A] == 1)
				origin -= right * movementSpeed * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_D] == 1)
				origin += right * movementSpeed * deltaTime;



			//Mouse Input

			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);


			switch (mouseState)
			{
			case SDL_BUTTON_LMASK:

				origin -= forward * (mouseY * movementSpeed/2 * deltaTime);
				totalYaw += mouseX * rotationSpeed * deltaTime;
				break;
			case SDL_BUTTON_RMASK:

				totalYaw += mouseX * rotationSpeed * deltaTime;
				totalPitch -= mouseY * rotationSpeed * deltaTime;
				break;
			case SDL_BUTTON_X2:

				origin.y -= mouseY * movementSpeed/2 * deltaTime;
				break;

			}


			Matrix finalRotation = Matrix::CreateRotationX(totalPitch) * Matrix::CreateRotationY(totalYaw);

			forward = finalRotation.TransformVector(Vector3::UnitZ);
			forward.Normalize();
		}

		void SetFovAngle(float newFovAngle)
		{
			float fov = newFovAngle * TO_RADIANS;
			fovAngle = tan(fov / 2.0f);
		}
	};
}
