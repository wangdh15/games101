//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <thread>
#include "Scene.hpp"
#include "Renderer.hpp"



inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";
    std::vector<std::thread> threads;
    threads.reserve(4);
    const int half_height = scene.height / 2, half_width = scene.width / 2;
    for (int j = 0; j < 2; ++j) {
        for (int i = 0; i < 2; ++i) {
            threads.emplace_back([&, i_idx=i, j_idx=j](){
                int cnt = 0;
                int all_cnt = half_height * half_width;
                for (int j = j_idx * half_height; j < (j_idx + 1) * half_height; ++j) {
                    for (int i = i_idx * half_width; i < (i_idx + 1) * half_width; ++i) {
                        // generate primary ray direction
                        float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                                imageAspectRatio * scale;
                        float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                        Vector3f dir = normalize(Vector3f(-x, y, 1));
                        for (int k = 0; k < spp; k++) {
                            framebuffer[j * scene.width + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                        }
                        ++cnt;
                        if (cnt % 2500 == 0)
                            std::cout << "thread " << i_idx * 2 + j_idx << ": Process " << cnt << "|" << all_cnt << std::endl;
                    }
                }
            });
        }
    }
    for (auto& thread : threads) {
        thread.join();
    }


    // for (uint32_t j = 0; j < scene.height; ++j) {
    //     for (uint32_t i = 0; i < scene.width; ++i) {
    //         // generate primary ray direction
    //         float x = (2 * (i + 0.5) / (float)scene.width - 1) *
    //                   imageAspectRatio * scale;
    //         float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

    //         Vector3f dir = normalize(Vector3f(-x, y, 1));
    //         for (int k = 0; k < spp; k++) {
    //             framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
    //         }
    //         m++;
    //     }
    //     UpdateProgress(j / (float)scene.height);
    // }
    // UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
