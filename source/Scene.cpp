#include "Scene.h"
#include "Utils.h"
#include "Material.h"
#include <algorithm>


namespace dae {

#pragma region Base Scene
	//Initialize Scene with Default Solid Color Material (RED)
	Scene::Scene():
		m_Materials({ new Material_SolidColor( colors::Red)})
	{

		m_SphereGeometries.reserve(32);
		m_PlaneGeometries.reserve(32);
		m_TriangleMeshGeometries.reserve(32);
		m_Lights.reserve(32);
	}

	Scene::~Scene()
	{
		for(auto& pMaterial : m_Materials)
		{
			delete pMaterial;
			pMaterial = nullptr;
		}

		m_Materials.clear();
	}

	void dae::Scene::GetClosestHit(const Ray& ray, HitRecord& closestHit) const
	{

		HitRecord hitRecord{};
		for (const auto& sphere : m_SphereGeometries)
		{
			if (GeometryUtils::HitTest_Sphere(sphere, ray, hitRecord))
			{
				if (hitRecord.t < closestHit.t)
					closestHit = hitRecord;
			}
		}

		for (const auto& plane : m_PlaneGeometries)
		{
			if (GeometryUtils::HitTest_Plane(plane, ray, hitRecord))
			{
				if (hitRecord.t < closestHit.t)
					closestHit = hitRecord;
			}
		}

		for (const auto& mesh : m_TriangleMeshGeometries)
		{
			if (GeometryUtils::HitTest_TriangleMesh(mesh, ray, hitRecord))
			{
				if (hitRecord.t < closestHit.t)
					closestHit = hitRecord;
			}
		}

	}

	bool Scene::DoesHit(const Ray& ray) const
	{

		for (const Sphere& s : m_SphereGeometries)
		{
			if (GeometryUtils::HitTest_Sphere(s, ray))
			{
				return true;
			}
		}

		for (const TriangleMesh& Tm : m_TriangleMeshGeometries)
		{
			if (GeometryUtils::HitTest_TriangleMesh(Tm, ray))
			{
				return true;
			}
		} 


		for (const Plane& p : m_PlaneGeometries)
		{
			if (GeometryUtils::HitTest_Plane(p, ray))
			{
				return true;
			}
		}

		return false;
	}

#pragma region Scene Helpers
	Sphere* Scene::AddSphere(const Vector3& origin, float radius, unsigned char materialIndex)
	{
		Sphere s;
		s.origin = origin;
		s.radius = radius;
		s.materialIndex = materialIndex;

		m_SphereGeometries.emplace_back(s);
		return &m_SphereGeometries.back();
	}

	Plane* Scene::AddPlane(const Vector3& origin, const Vector3& normal, unsigned char materialIndex)
	{
		Plane p;
		p.origin = origin;
		p.normal = normal;
		p.materialIndex = materialIndex;

		m_PlaneGeometries.emplace_back(p);
		return &m_PlaneGeometries.back();
	}

	TriangleMesh* Scene::AddTriangleMesh(TriangleCullMode cullMode, unsigned char materialIndex)
	{
		TriangleMesh m{};
		m.cullMode = cullMode;
		m.materialIndex = materialIndex;

		m_TriangleMeshGeometries.emplace_back(m);
		return &m_TriangleMeshGeometries.back();
	}

	Light* Scene::AddPointLight(const Vector3& origin, float intensity, const ColorRGB& color)
	{
		Light l;
		l.origin = origin;
		l.intensity = intensity;
		l.color = color;
		l.type = LightType::Point;

		m_Lights.emplace_back(l);
		return &m_Lights.back();
	}

	Light* Scene::AddDirectionalLight(const Vector3& direction, float intensity, const ColorRGB& color)
	{
		Light l;
		l.direction = direction;
		l.intensity = intensity;
		l.color = color;
		l.type = LightType::Directional;

		m_Lights.emplace_back(l);
		return &m_Lights.back();
	}

	unsigned char Scene::AddMaterial(Material* pMaterial)
	{
		m_Materials.push_back(pMaterial);
		return static_cast<unsigned char>(m_Materials.size() - 1);
	}
#pragma endregion
#pragma endregion

#pragma region SCENE W1
	void Scene_W1::Initialize()
	{
		//default: Material id0 >> SolidColor Material (RED)
		constexpr unsigned char matId_Solid_Red = 0;
		const unsigned char matId_Solid_Blue = AddMaterial(new Material_SolidColor{ colors::Blue });

		const unsigned char matId_Solid_Yellow = AddMaterial(new Material_SolidColor{ colors::Yellow });
		const unsigned char matId_Solid_Green = AddMaterial(new Material_SolidColor{ colors::Green });
		const unsigned char matId_Solid_Magenta = AddMaterial(new Material_SolidColor{ colors::Magenta });

		//Spheres
		AddSphere({ -25.f, 0.f, 100.f }, 50.f, matId_Solid_Red);
		AddSphere({ 25.f, 0.f, 100.f }, 50.f, matId_Solid_Blue);

		//Plane
		AddPlane({ -75.f, 0.f, 0.f }, { 1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 75.f, 0.f, 0.f }, { -1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 0.f, -75.f, 0.f }, { 0.f, 1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 75.f, 0.f }, { 0.f, -1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 0.f, 125.f }, { 0.f, 0.f,-1.f }, matId_Solid_Magenta);
	}
#pragma endregion

#pragma region SCENE W2
	void Scene_W2::Initialize()
	{
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.SetFovAngle(45.f);

		//default: Material id0 >> SolidColor Material (RED)
		constexpr unsigned char matId_Solid_Red = 0;
		const unsigned char matId_Solid_Blue = AddMaterial(new Material_SolidColor{ colors::Blue });

		const unsigned char matId_Solid_Yellow = AddMaterial(new Material_SolidColor{ colors::Yellow });
		const unsigned char matId_Solid_Green = AddMaterial(new Material_SolidColor{ colors::Green });
		const unsigned char matId_Solid_Magenta = AddMaterial(new Material_SolidColor{ colors::Magenta });

		//Plane


		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 10.f, 0.f }, { 0.f, -1.f,0.f }, matId_Solid_Yellow);

		AddPlane({ 0.f, 0.f, 10.f }, { 0.f, 0.f,-1.f }, matId_Solid_Magenta);

		AddPlane({ -5.f, 0.f, 0.f }, { 1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 5.f, 0.f, 0.f }, { -1.f, 0.f,0.f }, matId_Solid_Green);



		//Spheres
		AddSphere({ -1.75f, 1.f, 0.f }, .75f, matId_Solid_Red);
		AddSphere({ 0.f, 1.f, 0.f }, .75f, matId_Solid_Blue);

		AddSphere({ 1.75f, 1.f, 0.f }, .75f, matId_Solid_Red);
		AddSphere({ -1.75f, 3.f, 0.f }, .75f, matId_Solid_Blue);

		AddSphere({ 0.f, 3.f, 0.f }, .75f, matId_Solid_Red);
		AddSphere({ 1.75f, 3.f, 0.f }, .75f, matId_Solid_Blue);

		//Light
		AddPointLight({ 0.f, 5.f, -5.f }, 70.f, colors::White);

	}
#pragma endregion

#pragma region SCENE W3
	void Scene_W3::Initialize()
	{
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.SetFovAngle(45.f);

		const unsigned char matCT_GrayRoughMetal = AddMaterial(new Material_CookTorrence({.972f, .960f, .915f}, 1.f, 1.f));
		const unsigned char matCT_GrayMediumMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, 1.f, .6f));
		const unsigned char matCT_GraySmoothMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, 1.f, .1f));
		const unsigned char matCT_GrayRoughPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, 1.f));
		const unsigned char matCT_GrayMediumPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, .6f));
		const unsigned char matCT_GraySmoothPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, .1f));


		const unsigned char matLambert_GrayBlue = AddMaterial(new Material_Lambert({ .49f, 0.57f, 0.57f }, 1.f));

		//Plane 
		AddPlane({ 0.f, 0.f, 10.f }, { 0.f, 0.f, -1.f }, matLambert_GrayBlue); //Back
		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, matLambert_GrayBlue); //Bottom
		AddPlane({ 0.f, 10.f, 0.f }, { 0.f, -1.f, 0.f }, matLambert_GrayBlue); //Top
		AddPlane({ 5.f, 0.f, 0.f }, { -1.f, 0.f, 0.f }, matLambert_GrayBlue); //Right
		AddPlane({ -5.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }, matLambert_GrayBlue);	//Left

		//Spheres
		AddSphere({ -1.75f, 1.f, 0.f }, .75f, matCT_GrayRoughMetal);
		AddSphere({ 0.f, 1.f, 0.f }, .75f, matCT_GrayMediumMetal);
		AddSphere({ 1.75f, 1.f, 0.f }, .75f, matCT_GraySmoothMetal);
		AddSphere({ -1.75f, 3.f, 0.f }, .75f, matCT_GrayRoughPlastic);
		AddSphere({ 0.f, 3.f, 0.f }, .75f, matCT_GrayMediumPlastic);
		AddSphere({ 1.75f, 3.f, 0.f }, .75f, matCT_GraySmoothPlastic);

		//Light
		AddPointLight({ 0.f, 5.f, 5.f }, 50.f, {1.f, .61f, .45f}); //BackLight
		AddPointLight({ -2.5f, 5.f, -5.f }, 70.f, { 1.f, .8f, .45f }); //Front Light Left
		AddPointLight({ 2.5f, 2.5f, -5.f }, 50.f, { .34f, .47f, .68f });

	}
#pragma endregion

#pragma region SCENE W4
	void Scene_W4_TestScene::Initialize()
	{
		m_Camera.origin = { 0.f, 1.f, -5.f };
		m_Camera.SetFovAngle(45.f);

		const unsigned char matLambert_GrayBlue = AddMaterial(new Material_Lambert({ .49f, .57f, 0.57f }, 1.f));
		const unsigned char matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));
		

		//Plane 
		AddPlane({ 0.f, 0.f, 10.f }, { 0.f, 0.f, -1.f }, matLambert_GrayBlue); //Back
		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, matLambert_GrayBlue); //Bottom
		AddPlane({ 0.f, 10.f, 0.f }, { 0.f, -1.f, 0.f }, matLambert_GrayBlue); //Top
		AddPlane({ 5.f, 0.f, 0.f }, { -1.f, 0.f, 0.f }, matLambert_GrayBlue); //Right
		AddPlane({ -5.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }, matLambert_GrayBlue);	//Left


		//triangle temp
		auto triangle = Triangle{ {-0.75f, 0.5f, 0.f}, {-0.75, 2.f, 0.f}, {.75f, .5f, 0.f } };
		triangle.cullMode = TriangleCullMode::NoCulling;
		triangle.materialIndex = matLambert_White;
		m_Triangles.emplace_back(triangle);


		//Triangle Mesh
		m_pMesh = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		//pMesh->positions = { {-.75f, -1.f, .0f}, { -.75f, 1.f, .0f }, {.75f, 1.f, 1.f}, {.75f, -1.f, 0.f} };
		//pMesh->indices = {
		//	0,1,2, // Triangle 1
		//	0,2,3  // Triangle 2
		//};

		Utils::ParseOBJ("Resources/simple_cube.obj", m_pMesh->positions, m_pMesh->normals, m_pMesh->indices);


		m_pMesh->Scale({ .7f, .7f, .7f });
		m_pMesh->Translate({ 0.f, 2.f, 0.f });

		m_pMesh->UpdateTransforms();


		//Light
		AddPointLight({ 0.f, 5.f, 5.f }, 50.f, { 1.f, .61f, .45f }); //BackLight
		AddPointLight({ -2.5f, 5.f, -5.f }, 70.f, { 1.f, .8f, .45f }); //Front Light Left
		AddPointLight({ 2.5f, 2.5f, -5.f }, 50.f, { .34f, .47f, .68f });

	}
	void Scene_W4_TestScene::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);

		m_pMesh->RotateY(PI_DIV_2 / pTimer->GetTotal());

		m_pMesh->UpdateTransforms();

	}

#pragma endregion
	void Scene_W4_ReferenceScene::Initialize()
	{
		sceneName = "Reference Scene";
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.SetFovAngle(45.f);

		const unsigned char matCT_GrayRoughMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, 1.f, 1.f));
		const unsigned char matCT_GrayMediumMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, 1.f, .6f));
		const unsigned char matCT_GraySmoothMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, 1.f, .1f));
		const unsigned char matCT_GrayRoughPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, 1.f));
		const unsigned char matCT_GrayMediumPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, .6f));
		const unsigned char matCT_GraySmoothPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, .1f));

		const unsigned char matLambert_GrayBlue = AddMaterial(new Material_Lambert({ .49f, 0.57f, 0.57f }, 1.f));
		const unsigned char matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));


		//Plane 
		AddPlane({ 0.f, 0.f, 10.f }, { 0.f, 0.f, -1.f }, matLambert_GrayBlue); //Back
		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, matLambert_GrayBlue); //Bottom
		AddPlane({ 0.f, 10.f, 0.f }, { 0.f, -1.f, 0.f }, matLambert_GrayBlue); //Top
		AddPlane({ 5.f, 0.f, 0.f }, { -1.f, 0.f, 0.f }, matLambert_GrayBlue); //Right
		AddPlane({ -5.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }, matLambert_GrayBlue);	//Left

		//Spheres
		AddSphere({ -1.75f, 1.f, 0.f }, .75f, matCT_GrayRoughMetal);
		AddSphere({ 0.f, 1.f, 0.f }, .75f, matCT_GrayMediumMetal);
		AddSphere({ 1.75f, 1.f, 0.f }, .75f, matCT_GraySmoothMetal);
		AddSphere({ -1.75f, 3.f, 0.f }, .75f, matCT_GrayRoughPlastic);
		AddSphere({ 0.f, 3.f, 0.f }, .75f, matCT_GrayMediumPlastic);
		AddSphere({ 1.75f, 3.f, 0.f }, .75f, matCT_GraySmoothPlastic);


		//CW Winding order!

		const Triangle baseTriangle = { Vector3{ -.75f, 1.5f, 0.f}, Vector3{.75f, 0.f, 0.f}, Vector3{-.75f, .0f, 0.f} };
		m_Meshes[0] = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		m_Meshes[0]->AppendTriangle(baseTriangle, true);
		m_Meshes[0]->Translate({ -1.75f, 4.5f, 0.f });
		m_Meshes[0]->UpdateAABB();
		m_Meshes[0]->UpdateTransforms();

		m_Meshes[1] = AddTriangleMesh(TriangleCullMode::FrontFaceCulling, matLambert_White);
		m_Meshes[1]->AppendTriangle(baseTriangle, true);
		m_Meshes[1]->Translate({ 0.f, 4.5f, 0.f });
		m_Meshes[1]->UpdateAABB();
		m_Meshes[1]->UpdateTransforms();

		m_Meshes[2] = AddTriangleMesh(TriangleCullMode::NoCulling, matLambert_White);
		m_Meshes[2]->AppendTriangle(baseTriangle, true);
		m_Meshes[2]->Translate({ 1.75f, 4.5f, 0.f });
		m_Meshes[2]->UpdateAABB();
		m_Meshes[2]->UpdateTransforms();

		//Light
		AddPointLight({ 0.f, 5.f, 5.f }, 50.f, { 1.f, .61f, .45f }); //BackLight
		AddPointLight({ -2.5f, 5.f, -5.f }, 70.f, { 1.f, .8f, .45f }); //Front Light Left
		AddPointLight({ 2.5f, 2.5f, -5.f }, 50.f, { .34f, .47f, .68f });


	}
	void Scene_W4_ReferenceScene::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);
		const auto yawAngle = (cosf(pTimer->GetTotal()) + 1.0f) / 2.f * PI_2;

		for (const auto m : m_Meshes)
		{
			m->RotateY(yawAngle);
			m->UpdateTransforms();
		}
	}

	void Scene_W4_BunnyScene::Initialize()
	{
		sceneName = "Bunny Scene";
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.SetFovAngle( 45.f);


		const unsigned char matLambert_GrayBlue = AddMaterial(new Material_Lambert({ .49f, 0.57f, 0.57f }, 1.f));
		const unsigned char matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));


		//Plane 
		AddPlane({ 0.f, 0.f, 10.f }, { 0.f, 0.f, -1.f }, matLambert_GrayBlue); //Back
		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, matLambert_GrayBlue); //Bottom
		AddPlane({ 0.f, 10.f, 0.f }, { 0.f, -1.f, 0.f }, matLambert_GrayBlue); //Top
		AddPlane({ 5.f, 0.f, 0.f }, { -1.f, 0.f, 0.f }, matLambert_GrayBlue); //Right
		AddPlane({ -5.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }, matLambert_GrayBlue);	//Left


		m_pMesh = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		Utils::ParseOBJ("Resources/lowpoly_bunny.obj", m_pMesh->positions, m_pMesh->normals, m_pMesh->indices);


		m_pMesh->Scale({ 2.f, 2.f, 2.f });
		m_pMesh->RotateY(180);

		m_pMesh->slabTestOn = { true };
		m_pMesh->UpdateAABB();
		m_pMesh->UpdateTransforms();



		//Light
		AddPointLight({ 0.f, 5.f, 5.f }, 50.f, { 1.f, .61f, .45f }); //BackLight
		AddPointLight({ -2.5f, 5.f, -5.f }, 70.f, { 1.f, .8f, .45f }); //Front Light Left
		AddPointLight({ 2.5f, 2.5f, -5.f }, 50.f, { .34f, .47f, .68f });
	}

	void Scene_W4_BunnyScene::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);
		const auto yawAngle{ (cosf(pTimer->GetTotal()) + 1.f) / 2.f * PI_2 };
		m_pMesh->RotateY(yawAngle);
		m_pMesh->UpdateTransforms();
	}

}
