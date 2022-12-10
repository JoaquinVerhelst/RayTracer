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
			fovAngle{_fovAngle}
		{
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
		float rotationSpeed{30.f};

		bool isShiftPressed = false;


		Matrix CalculateCameraToWorld()
		{
			right = Vector3::Cross(Vector3::UnitY, forward).Normalized();
			up = Vector3::Cross(forward, right).Normalized();

			right.ToPoint4();
			up.ToPoint4();
			forward.ToPoint4();
			origin.ToPoint4();

			cameraToWorld = Matrix{ right, up, forward, origin };

			return cameraToWorld;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();



			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);
			if (pKeyboardState[SDL_SCANCODE_W] == 1)
				origin += movementSpeed * forward * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_S] == 1)
				origin -= movementSpeed * forward * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_A] == 1)
				origin -= right * movementSpeed * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_D] == 1)
				origin += right * movementSpeed * deltaTime;

			if (pKeyboardState[SDL_SCANCODE_LSHIFT] == 1 && !isShiftPressed)
			{
				movementSpeed *= 4;
				rotationSpeed *= 4;
				isShiftPressed = true;
			}

			if (pKeyboardState[SDL_SCANCODE_LSHIFT] == 0 && isShiftPressed)
			{
				movementSpeed /= 4;
				rotationSpeed /= 4;
				isShiftPressed = false;
			}

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

		

			if (mouseState & SDL_BUTTON(1) && mouseState & SDL_BUTTON(3))
			{
				if (mouseY >= 0)
					origin += movementSpeed * up * deltaTime;
				if (mouseY <= 0)
					origin -= movementSpeed * up * deltaTime;
			}
			else if (mouseState & SDL_BUTTON(1))
			{
				if (mouseY >= 0)
					origin += movementSpeed * forward * deltaTime;
				if (mouseY <= 0)
					origin -= movementSpeed * forward * deltaTime;
				if (mouseX >= 0)
					totalYaw += rotationSpeed * deltaTime;
				if (mouseX <= 0)
					totalYaw -= rotationSpeed * deltaTime;
			}
			else if (mouseState & SDL_BUTTON(3))
			{
				if (mouseY >= 0)
					totalPitch -= rotationSpeed * deltaTime;
				if (mouseY <= 0)
					totalPitch += rotationSpeed * deltaTime;

				if (mouseX >= 0)
					totalYaw += rotationSpeed * deltaTime;
				if (mouseX <= 0)
					totalYaw -= rotationSpeed * deltaTime;
			}


			Matrix finalRotation = Matrix::CreateRotationX(totalPitch) * Matrix::CreateRotationY(totalYaw);

			forward = finalRotation.TransformVector(Vector3::UnitZ);
			forward.Normalize();
		}
	};
}
