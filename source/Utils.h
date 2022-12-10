#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"
#include <cmath>


namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{

			float t0{}, t1{};

			float a = Vector3::Dot(ray.direction, ray.direction);
			float b = Vector3::Dot(2 * ray.direction, ray.origin - sphere.origin);
			float c = (Vector3::Dot(ray.origin - sphere.origin, ray.origin - sphere.origin)) - (sphere.radius * sphere.radius);

			float discriminant = b * b - 4 * a * c;

			if (discriminant < 0)
				return false;
			else if (discriminant == 0)
				t0 = -0.5f * b / a;
			else
			{
				float sqrtDiscriminant = sqrtf(discriminant);
				float q = (b > 0.0f) ? -0.5f * (b + sqrtDiscriminant) : -0.5f * (b - sqrtDiscriminant);

				t0 = q / a;
				t1 = c / q;
			}


			if (t0 > t1)
				std::swap(t0, t1);

			if (t0 < ray.min || t0 > ray.max)
			{
				t0 = t1;
				if (t0 < ray.min || t0 > ray.max)
					return false;
			}

			if (!ignoreHitRecord)
			{
				hitRecord.t = t0;
				hitRecord.didHit = true;
				hitRecord.materialIndex = sphere.materialIndex;
				hitRecord.origin = ray.origin + t0 * ray.direction;
				hitRecord.normal = ((ray.origin - sphere.origin) + (t0 * ray.direction)) / sphere.radius;
			}

			return true;

		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{

			float denominator = Vector3::Dot(ray.direction, plane.normal);

			if (fabs(denominator) > 0.00001f)
			{
				float t = Vector3::Dot(plane.origin - ray.origin, plane.normal) / denominator;

				if (t > ray.min && t < ray.max)
				{
					if (!ignoreHitRecord)
					{
						hitRecord.t = t;
						hitRecord.didHit = true;
						hitRecord.materialIndex = plane.materialIndex;
						hitRecord.origin = ray.origin + t * ray.direction;
						hitRecord.normal = plane.normal;
					}
					return true;
				}

				
			}

			return false;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{

			const Vector3 edgeV0V1 = triangle.v1 - triangle.v0;
			const Vector3 edgeV0V2 = triangle.v2 - triangle.v0;

			float NoRaydir = Vector3::Dot(triangle.normal, ray.direction);

			Vector3 pvec = Vector3::Cross(ray.direction, edgeV0V2);
			float det = Vector3::Dot(pvec, edgeV0V1);


			//Cullmode

			switch (triangle.cullMode)
			{
			case TriangleCullMode::BackFaceCulling:
				if (NoRaydir > 0)
					return false;
				break;
			case TriangleCullMode::FrontFaceCulling:
				if (NoRaydir < 0)
					return false;
				break;
			}


			float invDet = 1 / det;

			Vector3 tvec = ray.origin - triangle.v0;

			float u = Vector3::Dot( tvec, pvec) * invDet;

			if (u < 0 || u > 1) return false;

			Vector3 qvec = Vector3::Cross(tvec, edgeV0V1);

			float v = Vector3::Dot(ray.direction, qvec) * invDet;
			if (v < 0 || u + v > 1) return false;

			float t = Vector3::Dot(edgeV0V2, qvec) * invDet;


			if (!ignoreHitRecord)
			{
				hitRecord.t = t;
				hitRecord.didHit = true;
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.origin = ray.origin + t * ray.direction;
				hitRecord.normal = triangle.normal;
			}

			return true;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest

		inline bool SlabTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{

			float tx1 = (mesh.transformedminAABB.x - ray.origin.x) / ray.direction.x;
			float tx2 = (mesh.transformedMaxAABB.x - ray.origin.x) / ray.direction.x;

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			float ty1 = (mesh.transformedminAABB.y - ray.origin.y) / ray.direction.y;
			float ty2 = (mesh.transformedMaxAABB.y - ray.origin.y) / ray.direction.y;

			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));

			float tz1 = (mesh.transformedminAABB.z - ray.origin.z) / ray.direction.z;
			float tz2 = (mesh.transformedMaxAABB.z - ray.origin.z) / ray.direction.z;

			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmax > 0 && tmax >= tmin;
		}


		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{

			//SlabTest
			if (mesh.slabTestOn)
				if (!SlabTest_TriangleMesh(mesh, ray))
					return false;

			


			
			auto& pos = mesh.transformedPositions;

			
			HitRecord tempRecord;
			Triangle tempTriangle{};
			tempTriangle.cullMode = mesh.cullMode;
			tempTriangle.materialIndex = mesh.materialIndex;

			std::vector<Triangle> triangles{};

			int normalIdx{};

			for (int i = 0; i < mesh.indices.size(); i += 3)
			{
				tempTriangle.v0 = pos[mesh.indices[i]];
				tempTriangle.v1 = pos[mesh.indices[i + 1]];
				tempTriangle.v2 = pos[mesh.indices[i + 2]];
				tempTriangle.normal = mesh.transformedNormals[normalIdx];
				++normalIdx;

				if (HitTest_Triangle(tempTriangle, ray, hitRecord, ignoreHitRecord))
				{

					if (!tempRecord.didHit)
						tempRecord = hitRecord;
					else if (tempRecord.t < hitRecord.t)
						hitRecord = tempRecord;
					else
						tempRecord = hitRecord;

				}
			}

			
			
			if (!tempRecord.didHit)
				return false;

			hitRecord = tempRecord;

			return true;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{

			int normalIdx{};

			for (int i = 0; i < mesh.indices.size(); i += 3)
			{
				Triangle tempTriangle = { mesh.transformedPositions[mesh.indices[i]] ,mesh.transformedPositions[mesh.indices[i + 1]]
					,mesh.transformedPositions[mesh.indices[i + 2]], mesh.transformedNormals[normalIdx] };
				tempTriangle.cullMode = mesh.cullMode;
				tempTriangle.materialIndex = mesh.materialIndex;

				normalIdx++;

				if (HitTest_Triangle(tempTriangle, ray))
					return true;
				
			}


			return false;
		}


#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			return light.origin - origin;
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			ColorRGB Ergb = {};

			if (light.type == LightType::Point)
				Ergb = light.color * (light.intensity / (light.origin - target).SqrMagnitude());
			else if (light.type == LightType::Directional)
				Ergb = light.color * light.intensity;
			
			return Ergb;

		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof()) 
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if(isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
}

