#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>

#include "src/brdf.h"
#include "src/concurrency.h"
#include "src/gltf.h"
#include "src/scene.h"
#include "src/utils.h"
#include "src/vector.h"

float srgb(float x) { return std::pow(x, 1.f / 2.2f); }

Vector3 tonemapping(const Vector3 &color) 
{
  return Vector3(std::min(1.0f, color.x()), std::min(1.0f, color.y()),
                 std::min(1.0f, color.z()));
}

Vector3 tonemappingUncharted(const Vector3 &color) {
  const Vector3 A = Vector3(0.15f, 0.15f, 0.15f);
  const Vector3 B = Vector3(0.50f, 0.50f, 0.50f);
  const Vector3 C = Vector3(0.10f, 0.10f, 0.10f);
  const Vector3 D = Vector3(0.20f, 0.20f, 0.20f);
  const Vector3 E = Vector3(0.02f, 0.02f, 0.02f);
  const Vector3 F = Vector3(0.30f, 0.30f, 0.30f);
  const Vector3 wPoint = Vector3(11.20f, 11.30f, 11.20f);

  auto applay = [&](const Vector3 &c) {
    return ((c * (A * c + C * B) + D * E) / (c * (A * c + B) + D * F)) - E / F;
  };

  return applay(color) * (Vector3(1.0, 1.0f, 1.0f) / applay(wPoint));
}

void saveImageToFile(std::uint16_t width, std::uint16_t height,
                     const std::vector<Vector3> &data) {
  std::ofstream outfile("output.ppm", std::ios::out | std::ios::binary);

  if (outfile.is_open()) {
    outfile << "P3\n" << width << " " << height << "\n255\n";

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        Vector3 color = tonemappingUncharted(data[y * width + x]);
        int r = (int)std::clamp(srgb(color.x()) * 255, 0.0f, 255.0f); // R
        int g = (int)std::clamp(srgb(color.y()) * 255, 0.0f, 255.0f); // G
        int b = (int)std::clamp(srgb(color.z()) * 255, 0.0f, 255.0f); // B

        outfile << r << " " << g << " " << b << " ";
      }
      outfile << "\n";
    }
    outfile.close();

    // Сообщаем о сохранении
    printf("Image saved to output.ppm\n");
  } else {
    printf("Error: Could not open output.ppm for writing.\n");
  }
}

// можно использовать точку и дистанцию
float intersectPlane(const math::Ray &ray, Vector3 poinOnPlane,
                     Vector3 normPlane, float tMin, float tMax) 
{
  float t = dot((poinOnPlane - ray.origin), normPlane) /
            dot(ray.direction, normPlane);
  if (t > tMin && t < tMax) {
    return t;
  } else {
    return tMax;
  }
}

// Vector3 getUniformSampleOffset( int index, int side_count )
//{
//	const float haflDist = 0.5 / side_count;
//	const float dist = 1.0 / side_count;
//	const float x = index % side_count;
//	const float y = std::floor( index / side_count );
//	return Vector3( haflDist + x * dist, haflDist + y * dist, 0.0 );
// }

Vector3 getUniformSampleOffset(int index, int side_count) 
{
  const float x_idx = (float)(index % side_count);
  const float y_idx = (float)std::floor(index / side_count);

  const float dist = 1.0f / side_count;

  const float jitterX = randomFloat();
  const float jitterY = randomFloat();

  const float u = (x_idx + jitterX) * dist;
  const float v = (y_idx + jitterY) * dist;

  return Vector3(u, v, 0.0f);
}

Vector3 randomUniformVectorHemispher() 
{
  const float phi = randFloat(0, 1) * 2.0f * PI;
  const float cosTheta = randFloat(0, 1) * 2.0f - 1.0f;
  const float sinTheta = std::sqrt(1 - cosTheta * cosTheta);
  const float x = std::cos(phi) * sinTheta;
  const float y = cosTheta;
  const float z = std::sin(phi) * sinTheta;
  return Vector3(x, y, z);
}

Vector3 randOnHemispher(const Vector3 &normal) 
{
  Vector3 onSphere = randUnitVector();
  if (dot(onSphere, normal) > 0.0f)
    return onSphere;
  else
    return -onSphere;
}

Vector3 reflect(const Vector3 &d, const Vector3 &n) 
{
  return d - 2.0f * dot(d, n) * n;
}

Vector3 trace(const math::Ray &ray, const Scene &scene, int depth) 
{
  const float tMin = 0.1f;
  float tMax = 10000;
  Vector3 hitNormal;

  int matIndex = -1;

  math::Triangle tr;
  float t = scene.intersect(ray, tMin, tMax, tr);
  if (t < tMax) {
    hitNormal = unit_vector(cross(tr.b - tr.a, tr.c - tr.a));
    tMax = t;
    matIndex = tr.matIndex;
  }
  if (tMax == 10000 || matIndex == -1)
    return Vector3(0.f, 0.f, 0.f); // scene.enviroment();

  if (dot(hitNormal, ray.direction) > 0.0)
    hitNormal = -hitNormal;

  const Material m = scene.materials()[matIndex];

  // float probToContinue = 0.5;// std::min(0.9f, std::max( 1e-3f, std::max(
  // m.albedo.x(), std::max( m.albedo.y(), m.albedo.z() ) )));
  const float probToContinue =
      std::max(m.albedo.x(), std::max(m.albedo.y(), m.albedo.z()));
  const int maxDepth = 10;
  if (depth > maxDepth && (randFloat(0, 1) > probToContinue))
    return m.emission;

  Vector3 color;

  auto newDir = randomUniformVectorHemispher();
  float cosTheta = dot(newDir, hitNormal);
  if (cosTheta < 0.0) {
    newDir *= -1;
    cosTheta *= -1;
  }
  const Vector3 newOrig = ray.origin + ray.direction * tMax + newDir * 1e-4f;
  const math::Ray newRay({newOrig, newDir});

  // float brdf = 1.0f / PI;
  // float pdf = 1.0f / ( 2.0f * PI );
  const Vector3 L = newDir;
  const Vector3 V = ray.direction * -1.0f;
  const Vector3 H = unit_vector((L + V) * 0.5f);
  const Vector3 N = hitNormal;
  auto brdf = BRDF(m.albedo, m.metallic, m.roughness, L, H, N, V);
  float pdf = 1.0f / (2.0f * PI);

  color = trace(newRay, scene, depth + 1) * brdf * cosTheta / pdf + m.emission;

  if (depth > maxDepth)
    return color * (1.0f / probToContinue);

  return color;
}

// Vector3 trace_iterative( math::Ray ray, const Scene& scene, int maxDepth)
//{
//	Vector3 throughput = Vector3(1.0, 1.0, 1.0);
//	Vector3 radiance = Vector3(0.0, 0.0, 0.0);
//
//	const float tMin = 0.001f;
//
//	for (int depth = 0; depth < maxDepth; ++depth)
//	{
//		float tMax = 10000.0f;
//		float t;
//		Vector3 hitNormal;
//		int matIndex = -1;
//
//
//		for (const auto& sp : scene.spheres())
//		{
//			t = math::intersect(ray, sp, tMin, tMax);
//			if (t < tMax)
//			{
//				Vector3 pos = ray.origin + ray.direction * t;
//				hitNormal = unit_vector(pos - sp.pos);
//				tMax = t;
//				matIndex = sp.matIndex;
//			}
//		}
//
//		for (const auto& p : scene.planes())
//		{
//			t = intersectPlane2(ray, p.normal, p.dist, tMin, tMax);
//			if (t < tMax)
//			{
//				hitNormal = p.normal;
//				tMax = t;
//				matIndex = p.matIndex;
//			}
//		}
//
//		for (const auto& tr : scene.triangles())
//		{
//			t = math::intersect(ray, tr, tMin, tMax);
//			if (t < tMax)
//			{
//				hitNormal = unit_vector(cross(tr.b - tr.a, tr.c
//- tr.a)); 				tMax = t;
//matIndex = tr.matIndex;
//			}
//		}
//
//		if (matIndex == -1 || tMax == 10000.0f) // matIndex == -1 -
// более явная проверка на промах
//		{
//			radiance += throughput * scene.enviroment();
//			break;
//		}
//
//		if (dot(hitNormal, ray.direction) > 0.0)
//			hitNormal = -hitNormal;
//
//
//		Vector3 newDir = randomUniformVectorHemispher();
//
//		auto cosTheta = dot(newDir, hitNormal);
//		if (cosTheta < 0.0)
//			newDir *= -1;
//
//		cosTheta = dot(newDir, hitNormal);
//
//		const Material m = scene.materials()[matIndex];
//
//
//		radiance += throughput * m.emission;
//
//		const float brdf = 1.0f / PI;
//		const float pdf = 1.0f / (2.0f * PI);
//
//		throughput = throughput * m.albedo * (brdf * cosTheta / pdf);
//
//
//		const Vector3 newOrig = ray.origin + ray.direction * tMax +
// newDir * 1e-4f;
//
//		ray.origin = newOrig;
//		ray.direction = newDir;
//	}
//
//	return radiance;
// }

std::atomic<int> completed_pixels(0);

void display_progress(int total_pixels) {
  const int BAR_LENGTH = 50;
  int last_percentage = -1;

  while (completed_pixels.load() < total_pixels) {
    int current = completed_pixels.load();

    int percentage = (current * 100) / total_pixels;
    int filled_length = (percentage * BAR_LENGTH) / 100;

    if (percentage > last_percentage) {
      std::cout << "\r[";

      for (int i = 0; i < filled_length; ++i) {
        std::cout << "#";
      }

      for (int i = filled_length; i < BAR_LENGTH; ++i) {
        std::cout << " ";
      }

      std::cout << "] " << percentage << "% (" << current << "/" << total_pixels
                << ")";
      std::cout.flush();
      last_percentage = percentage;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::cout << "\r[";
  for (int i = 0; i < BAR_LENGTH; ++i)
    std::cout << "#";
  std::cout << "] 100% (" << total_pixels << "/" << total_pixels << ")\n";
  std::cout.flush();
}

int main() {
  Scene scene;
  //gltf::parse("../scenes/07-scene-easy.gltf", scene);
  gltf::parse("../scenes/07-scene-medium-2.gltf", scene);

  const float aspectRatio = scene.camera().aspectRatio;
  const std::uint16_t width = 600;
  const std::uint16_t height = width / aspectRatio;

  auto &camera = scene.camera();
  const Vector3 camerForward = unit_vector(camera.target - camera.pos);
  const Vector3 camerRight = unit_vector(cross(camerForward, camera.up));
  const Vector3 camerUp = cross(camerRight, camerForward);

  const float pixSize = 1.0f / height;
  // const float viewportHight = 2.0f * std::tan( (camera.fov / 180.0f * PI) *
  // 0.5f );
  const float viewportHight = 2.0f * std::tan((camera.fov) * 0.5f);

  //	const Vector3 leftTop( -aspectRatio / 2, 0.5f, 1.0f );
  const Vector3 leftTop(-aspectRatio * viewportHight / 2.0f,
                        viewportHight / 2.0f, 1.0f);

  std::vector<Vector3> data;
  data.resize(width * height);

  // const int SIDE_SAMPLE_COUNT = scene.samples();
  const int SIDE_SAMPLE_COUNT = 8;
  auto start = std::chrono::high_resolution_clock::now();

  TaskManager manager(8, 32);
  
  completed_pixels = 0;
  std::thread progress_thread(display_progress, (int)data.size());

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      while (!manager.add(
          [&](int x, int y, std::vector<Vector3> &data, const Scene &scene) {
            Vector3 color(0, 0, 0);
            const float u = float(x) / width;
            const float v = float(y) / height;

            for (int s = 0; s < SIDE_SAMPLE_COUNT * SIDE_SAMPLE_COUNT; ++s) {
              // Vector3 pixPos = leftTop + Vector3( pixSize / 2.0 + x *
              // pixSize, pixSize / 2.0 + y * pixSize, 0 ); Vector3 pixPos =
              // leftTop + Vector3( pixSize / 2.0f + u * aspectRatio, -pixSize
              // / 2.0f - v, 0.0f );
              const Vector3 offset = getUniformSampleOffset(s, SIDE_SAMPLE_COUNT);
              const Vector3 pixPosVS = leftTop + Vector3((pixSize * offset.x() + u * aspectRatio) * viewportHight, (-pixSize * offset.y() - v) * viewportHight, 0.0f);
              const Vector3 pixPos = camera.pos + pixPosVS.x() * camerRight + pixPosVS.y() * camerUp + pixPosVS.z() * camerForward;

              const Vector3 dir = unit_vector(pixPos - camera.pos);
              const math::Ray ray({camera.pos, dir});
              // color += trace_iterative( ray, scene, 4 );
              color += trace(ray, scene, 0);
            }

            data[y * width + x] =
                color / float(SIDE_SAMPLE_COUNT * SIDE_SAMPLE_COUNT);
            completed_pixels.fetch_add(1);
          },
          x, y, std::ref(data), std::cref(scene))) {
        std::this_thread::yield();
      }
    }
  }
  manager.stop();
  if (progress_thread.joinable()) {
    progress_thread.join();
  }
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  std::cout << "Time: " << duration_ms.count() << " milliseconds" << std::endl;

  saveImageToFile(width, height, data);
  
  return 0;
}