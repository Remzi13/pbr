#define SDL_MAIN_HANDLED
#include <SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <atomic>
#include <thread>

//#include <unistd.h>
#include <cstdio>
#include "src/brdf.h"
#include "src/concurrency.h"
#include "src/gltf.h"
#include "src/scene.h"
#include "src/utils.h"
#include "src/vector.h"

float srgb(float x) { return std::pow(x, 1.f / 2.2f); }

Vector3 tonemappingUncharted(const Vector3& color) {
	const Vector3 A = Vector3(0.15f, 0.15f, 0.15f);
	const Vector3 B = Vector3(0.50f, 0.50f, 0.50f);
	const Vector3 C = Vector3(0.10f, 0.10f, 0.10f);
	const Vector3 D = Vector3(0.20f, 0.20f, 0.20f);
	const Vector3 E = Vector3(0.02f, 0.02f, 0.02f);
	const Vector3 F = Vector3(0.30f, 0.30f, 0.30f);
	const Vector3 wPoint = Vector3(11.20f, 11.30f, 11.20f);

	auto applay = [&](const Vector3& c) {
		return ((c * (A * c + C * B) + D * E) / (c * (A * c + B) + D * F)) - E / F;
		};

	return applay(color) * (Vector3(1.0, 1.0f, 1.0f) / applay(wPoint));
}

void saveImageToFile(std::uint16_t width, std::uint16_t height,
	const std::vector<Vector3>& data) {
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
		printf("Image saved to output.ppm\n");
	}
	else {
		printf("Error: Could not open output.ppm for writing.\n");
	}
}

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

Vector3 trace(const math::Ray& ray, const Scene& scene, int depth)
{
	const float tMin = 0.1f;
	float tMax = 10000;
	Vector3 hitNormal;

	auto [t, tr, mat] = scene.intersect(ray, tMin, tMax);
	if (t < tMax) {
		tMax = t;

		const Vector3 p = ray.origin + ray.direction * tMax;
		const Vector3 v0 = tr.b - tr.a;
		const Vector3 v1 = tr.c - tr.a;
		const Vector3 v2 = p - tr.a;

		const float d00 = dot(v0, v0);
		const float d01 = dot(v0, v1);
		const float d11 = dot(v1, v1);
		const float d20 = dot(v2, v0);
		const float d21 = dot(v2, v1);

		const float denom = d00 * d11 - d01 * d01;
		if (std::abs(denom) < 1e-8f) {
			hitNormal = unit_vector(cross(v0, v1));
		}
		else {
			const float v = (d11 * d20 - d01 * d21) / denom;
			const float w = (d00 * d21 - d01 * d20) / denom;
			const float u = 1.0f - v - w;

			hitNormal = unit_vector(u * tr.na + v * tr.nb + w * tr.nc);
		}
	}
	if (tMax == 10000)
		return Vector3(0.f, 0.f, 0.f);

	if (dot(hitNormal, ray.direction) > 0.0)
		hitNormal = -hitNormal;


	const float probToContinue =
		std::max(mat.albedo.x(), std::max(mat.albedo.y(), mat.albedo.z()));
	const int maxDepth = 10;
	if (depth > maxDepth && (randFloat(0, 1) > probToContinue))
		return mat.emission;

	Vector3 color;

	auto newDir = randomUniformVectorHemispher();
	float cosTheta = dot(newDir, hitNormal);
	if (cosTheta < 0.0) {
		newDir *= -1;
		cosTheta *= -1;
	}
	const Vector3 newOrig = ray.origin + ray.direction * tMax + newDir * 1e-4f;
	const math::Ray newRay({ newOrig, newDir });

	const Vector3 L = newDir;
	const Vector3 V = ray.direction * -1.0f;
	const Vector3 H = unit_vector((L + V) * 0.5f);
	const Vector3 N = hitNormal;
	auto brdf = BRDF(mat.albedo, mat.metallic, mat.roughness, L, H, N, V);
	float pdf = 1.0f / (2.0f * PI);

	color = trace(newRay, scene, depth + 1) * brdf * cosTheta / pdf + mat.emission;

	if (depth > maxDepth)
		return color * (1.0f / probToContinue);

	return color;
}

std::atomic<int> completed_pixels(0);

struct RenderSettings {
	int width = 600;
	int height = 400;
	int samples = 1;
	bool rendering = false;
};

int main(int argc, char* argv[]) {
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
		printf("Error: %s\n", SDL_GetError());
		return -1;
	}

	SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
	SDL_Window* window = SDL_CreateWindow("PBR Path Tracer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
	if (renderer == nullptr) {
		SDL_Log("Error creating SDL_Renderer!");
		return 0;
	}

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	//io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

	ImGui::StyleColorsDark();

	ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
	ImGui_ImplSDLRenderer2_Init(renderer);

	Scene scene;
	const char* scenePath = "scenes/07-scene-medium-2.gltf";
	const char* fallbackPath = "../scenes/07-scene-medium-2.gltf";
	bool loaded = gltf::parse(scenePath, scene);
	if (!loaded) {
		printf("Failed to load %s, trying %s\n", scenePath, fallbackPath);
		loaded = gltf::parse(fallbackPath, scene);
	}

	RenderSettings settings;
	settings.width = 600;

	if (loaded && scene.camera().aspectRatio > 0.01f) {
		settings.height = (int)(settings.width / scene.camera().aspectRatio);
	}
	else {
		printf("Warning: Scene not loaded or camera invalid. Using default settings.\n");
		settings.height = 400;
		Camera defaultCam;
		defaultCam.pos = Vector3(0, 0, 5);
		defaultCam.target = Vector3(0, 0, 0);
		defaultCam.up = Vector3(0, 1, 0);
		defaultCam.fov = 45.0f * PI / 180.0f;
		defaultCam.aspectRatio = 1.5f;
		scene.setCamera(defaultCam);
	}

	std::vector<Vector3> renderData(settings.width * settings.height, Vector3(0, 0, 0));
	SDL_Texture* renderTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, settings.width, settings.height);
	if (!renderTexture) {
		printf("Error: Could not create SDL_Texture (%dx%d): %s\n", settings.width, settings.height, SDL_GetError());
	}
	std::vector<uint32_t> pixels(settings.width * settings.height, 0xFF000000);

	struct NodeState {
		float translation[3] = { 0,0,0 };
	};
	std::vector<NodeState> nodeStates(scene.nodes().size());

	TaskManager taskManager(8, 32);

	auto startRender = [&]() {
		printf("startRender triggered\n");
		if (settings.width <= 0 || settings.height <= 0) return;
		completed_pixels = 0;
		settings.rendering = true;
		renderData.assign(settings.width * settings.height, Vector3(0, 0, 0));

		// Use a separate thread to submit tasks to the manager to avoid blocking UI
		std::thread([&]() {
			auto& camera = scene.camera();
			const Vector3 camerForward = unit_vector(camera.target - camera.pos);
			const Vector3 camerRight = unit_vector(cross(camerForward, camera.up));
			const Vector3 camerUp = cross(camerRight, camerForward);
			const float pixSize = 1.0f / settings.height;
			const float viewportHight = 2.0f * std::tan((camera.fov) * 0.5f);
			const float aspectRatio = camera.aspectRatio;
			const Vector3 leftTop(-aspectRatio * viewportHight / 2.0f, viewportHight / 2.0f, 1.0f);

			for (int y = 0; y < settings.height; ++y) {
				// Submit row-based tasks for better performance
				// Capture locals by value, but shared resources by reference
				while (!taskManager.add([=, &renderData, &scene, &settings](int row) {
					for (int x = 0; x < settings.width; ++x) {
						Vector3 color(0, 0, 0);
						const float u = float(x) / settings.width;
						const float v = float(row) / settings.height;
						const int SIDE_SAMPLE_COUNT = settings.samples;

						for (int s = 0; s < SIDE_SAMPLE_COUNT * SIDE_SAMPLE_COUNT; ++s) {
							const Vector3 offset = getUniformSampleOffset(s, SIDE_SAMPLE_COUNT);
							const Vector3 pixPosVS = leftTop + Vector3((pixSize * offset.x() + u * aspectRatio) * viewportHight, (-pixSize * offset.y() - v) * viewportHight, 0.0f);
							const Vector3 pixPos = camera.pos + pixPosVS.x() * camerRight + pixPosVS.y() * camerUp + pixPosVS.z() * camerForward;

							const Vector3 dir = unit_vector(pixPos - camera.pos);
							const math::Ray ray({ camera.pos, dir });
							color += trace(ray, scene, 0);
						}
						renderData[row * settings.width + x] = color / float(SIDE_SAMPLE_COUNT * SIDE_SAMPLE_COUNT);
					}
					completed_pixels.fetch_add(settings.width);
					if (row % 50 == 0) {
						printf("Finished row %d\n", row);
					}
					}, y)) {
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
			}
			}).detach();
		};

	bool done = false;
	while (!done) {
		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			ImGui_ImplSDL2_ProcessEvent(&event);
			if (event.type == SDL_QUIT)
				done = true;
			if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window))
				done = true;
		}

		ImGui_ImplSDLRenderer2_NewFrame();
		ImGui_ImplSDL2_NewFrame();
		ImGui::NewFrame();

		{
			ImGui::Begin("Settings");
			ImGui::Text("Resolution: %d x %d", settings.width, settings.height);
			ImGui::SliderInt("Samples", &settings.samples, 1, 16);
			if (ImGui::Button("Start Render") && !settings.rendering) {
				startRender();
			}
			if (settings.rendering) {
				float progress = (float)completed_pixels.load() / (settings.width * settings.height);
				ImGui::ProgressBar(progress);
				if (completed_pixels.load() == settings.width * settings.height) {
					settings.rendering = false;
				}
			}
			if (ImGui::Button("Save Image") && completed_pixels.load() > 0) {
				saveImageToFile(settings.width, settings.height, renderData);
			}
			ImGui::End();
		}

		{
			ImGui::Begin("Scene Editor");
			if (ImGui::CollapsingHeader("Materials")) {
				auto& materials = scene.materials();
				for (size_t i = 0; i < materials.size(); ++i) {
					ImGui::PushID((int)i);
					ImGui::Text("Material %zu: %s", i, materials[i].name.c_str());
					bool changed = false;
					changed |= ImGui::ColorEdit3("Albedo", (float*)&materials[i].albedo);
					changed |= ImGui::ColorEdit3("Emission", (float*)&materials[i].emission);
					changed |= ImGui::SliderFloat("Metallic", &materials[i].metallic, 0.0f, 1.0f);
					changed |= ImGui::SliderFloat("Roughness", &materials[i].roughness, 0.0f, 1.0f);

					if (changed && settings.rendering) {
						startRender();
					}

					ImGui::Separator();
					ImGui::PopID();
				}
			}
			if (ImGui::CollapsingHeader("Objects")) {
				auto& nodes = scene.nodes();
				auto& materials = scene.materials();
				for (size_t i = 0; i < nodes.size(); ++i) {
					ImGui::PushID((int)(i + 1000));
					ImGui::Text("Node: %s", nodes[i].name.c_str());

					int matIdx = 0;
					if (!nodes[i].bvh.primitives().empty()) {
						matIdx = (int)nodes[i].matIndex;
					}

					if (ImGui::BeginCombo("Material", materials[matIdx].name.empty() ? ("Material " + std::to_string(matIdx)).c_str() : materials[matIdx].name.c_str())) {
						for (int m = 0; m < (int)materials.size(); m++) {
							bool isSelected = (matIdx == m);
							std::string matName = materials[m].name;
							if (matName.empty()) matName = "Material " + std::to_string(m);
							if (ImGui::Selectable(matName.c_str(), isSelected)) {
								nodes[i].matIndex = (size_t)m;
								nodes[i].rebuild();
								if (settings.rendering) startRender();
							}
							if (isSelected) ImGui::SetItemDefaultFocus();
						}
						ImGui::EndCombo();
					}

					float currentPos[3] = { nodeStates[i].translation[0], nodeStates[i].translation[1], nodeStates[i].translation[2] };
					if (ImGui::DragFloat3("Translation", currentPos, 0.1f)) {
						Vector3 delta(currentPos[0] - nodeStates[i].translation[0],
							currentPos[1] - nodeStates[i].translation[1],
							currentPos[2] - nodeStates[i].translation[2]);

						nodeStates[i].translation[0] = currentPos[0];
						nodeStates[i].translation[1] = currentPos[1];
						nodeStates[i].translation[2] = currentPos[2];

						for (auto& tr : nodes[i].bvh.primitives()) {
							tr.a += delta;
							tr.b += delta;
							tr.c += delta;
						}
						nodes[i].rebuild();
						// Reset rendering if something moved
						if (settings.rendering) {
							startRender();
						}
					}
					ImGui::Separator();
					ImGui::PopID();
				}
			}
			ImGui::End();
		}

		// Update texture
		bool shouldUpdate = true; // Always update for now to avoid initial garbage
		if (renderTexture && shouldUpdate) {
			static bool firstUpdate = true;
			if (firstUpdate) {
				printf("Starting first texture update...\n");
				firstUpdate = false;
			}
			bool hasWarned = false;
			for (int i = 0; i < settings.width * settings.height; ++i) {
				Vector3 c = renderData[i];
				// Check for NaN or Inf to avoid white screen
				if (!std::isfinite(c.x()) || !std::isfinite(c.y()) || !std::isfinite(c.z())) {
					if (!hasWarned) {
						printf("Warning: NaN or Inf detected in renderData[0]\n");
						hasWarned = true;
					}
					c = Vector3(1, 0, 1); // Pink for error
				}
				Vector3 color = tonemappingUncharted(c);
				uint8_t r = (uint8_t)std::clamp(srgb(color.x()) * 255.0f, 0.0f, 255.0f);
				uint8_t g = (uint8_t)std::clamp(srgb(color.y()) * 255.0f, 0.0f, 255.0f);
				uint8_t b = (uint8_t)std::clamp(srgb(color.z()) * 255.0f, 0.0f, 255.0f);
				pixels[i] = (0xFF << 24) | (b << 16) | (g << 8) | r;
			}
			if (SDL_UpdateTexture(renderTexture, nullptr, pixels.data(), settings.width * sizeof(uint32_t)) != 0) {
				printf("SDL_UpdateTexture failed: %s\n", SDL_GetError());
			}
			else {
				static int updateCount = 0;
				if (updateCount++ < 5) {
					printf("Update texture %d: pixels[0]=%08X\n", updateCount, pixels[0]);
				}
			}
		}

		ImGui::Begin("Viewport");
		ImVec2 viewportSize = ImGui::GetContentRegionAvail();
		if (viewportSize.x > 0 && viewportSize.y > 0) {
			ImGui::Image((ImTextureID)renderTexture, viewportSize);
		}
		else {
			ImGui::Text("Viewport too small");
		}
		ImGui::End();

		ImGui::Render();
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
		SDL_RenderClear(renderer);
		ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
		SDL_RenderPresent(renderer);
	}

	taskManager.stop();
	ImGui_ImplSDLRenderer2_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();

	SDL_DestroyTexture(renderTexture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();

	return 0;
}