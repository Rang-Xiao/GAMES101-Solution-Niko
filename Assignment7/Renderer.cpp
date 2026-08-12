//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"

// 多线程相关头文件，C++11标准引入了线程库，提供了std::thread和std::mutex等类来支持多线程编程=
#include <thread>
#include <mutex>


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.0001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.


// 进度条用的互斥锁（可选，只是为了让进度输出不乱）
std::mutex progress_mutex;

// 每个线程执行的函数，负责渲染 [row_start, row_end) 这些行
#include <atomic>

std::atomic<int> finished_rows(0);

void para(Vector3f eye_pos, std::vector<Vector3f>& framebuffer, const Scene& scene,
    int spp, float imageAspectRatio, float scale, int row_start, int row_end)
{
    for (int j = row_start; j < row_end; ++j) {
        for (int i = 0; i < scene.width; ++i) {
            float x = (2 * (i + 0.5f) / (float)scene.width - 1) *
                imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5f) / (float)scene.height) * scale;
            Vector3f dir = normalize(Vector3f(-x, y, 1));

            int m = j * scene.width + i;
            for (int k = 0; k < spp; k++) {
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
            }
        }
        // 每完成一行，原子+1，然后更新进度
        int done = ++finished_rows;
        UpdateProgress(done / (float)scene.height);
    }
}

void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);
    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    int spp = 480;
    int thread_num = 32;
    int thread_step = scene.height / thread_num;

    std::cout << "SPP: " << spp << "\n";
    std::cout << "Thread: " << thread_num << "\n";

    std::vector<std::thread> threads;
    for (int i = 0; i < thread_num; i++) {
        int row_start = i * thread_step;
        int row_end = (i == thread_num - 1) ? scene.height : (i + 1) * thread_step;
        threads.push_back(std::thread(para, eye_pos, std::ref(framebuffer),
            std::ref(scene), spp, imageAspectRatio, scale,
            row_start, row_end));
    }
    for (auto& t : threads)
        t.join();

    UpdateProgress(1.f);

    // save framebuffer to file（不变）
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
