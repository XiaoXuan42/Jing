#include <CoreLayer/Math/Math.h>
#include <FunctionLayer/Camera/Pinhole.h>
#include <FunctionLayer/Integrator/Integrator.h>
#include <FunctionLayer/Sampler/Sampler.h>
#include <FunctionLayer/Scene/Scene.h>
#include <FunctionLayer/Texture/Mipmap.h>
#include <ResourceLayer/Factory.h>
#include <ResourceLayer/FileUtil.h>
#include <ResourceLayer/Image.h>
#include <ResourceLayer/JsonUtil.h>
#include <stdio.h>

#include <chrono>
#include <fstream>
#include <regex>

#include <openvdb/openvdb.h>

#ifdef MOER_OPENMP
#include <omp.h>
#endif

int main(int argc, char **argv) {
    openvdb::initialize();
    const std::string sceneDir = std::string(argv[1]);
    FileUtil::setWorkingDirectory(sceneDir);
    std::string sceneJsonPath = FileUtil::getFullPath("scene.json");
    std::ifstream fstm(sceneJsonPath);
    Json json = Json::parse(fstm);
    auto camera = Factory::construct_class<Camera>(json["camera"]);
    auto scene = std::make_shared<Scene>(json["scene"]);
    auto integrator = Factory::construct_class<Integrator>(json["integrator"]);
    auto sampler = Factory::construct_class<Sampler>(json["sampler"]);
    int spp = sampler->xSamples * sampler->ySamples;
    int width = camera->film->size[0], height = camera->film->size[1];

    auto start = std::chrono::system_clock::now();

#ifdef MOER_OPENMP
#pragma omp parallel for num_threads(12)
    for (int i = 0; i < height * width; ++i) {
        int y = i / width;
        int x = i - y * width;
        Vector2f NDC{(float)x / width, (float)y / height};
        Spectrum li(.0f);
        for (int j = 0; j < spp; ++j) {
            Ray ray = camera->sampleRayDifferentials(
                CameraSample{sampler->next2D()}, NDC);
            li += integrator->li(ray, *scene, sampler);
        }
        camera->film->deposit({x, y}, li / spp);
    }
#else
    for (int i = 0; i < height * width; ++i) {
        int y = i / width;
        int x = i - y * width;
        Vector2f NDC{(float)x / width, (float)y / height};
        Spectrum li(.0f);
        for (int j = 0; j < spp; ++j) {
            Ray ray = camera->sampleRayDifferentials(
                CameraSample{sampler->next2D()}, NDC);
            li += integrator->li(ray, *scene, sampler);
        }
        camera->film->deposit({x, y}, li / spp);
    }
#endif

    auto end = std::chrono::system_clock::now();

    printf("\nRendering costs %.2fs\n",
           (std::chrono::duration_cast<std::chrono::milliseconds>(end - start))
                   .count() /
               1000.f);

    //* 目前支持输出为png/hdr两种格式
    std::string outputName =
        fetchRequired<std::string>(json["output"], "filename");
    if (std::regex_match(outputName, std::regex("(.*)(\\.png)"))) {
        camera->film->savePNG(outputName.c_str());
    } else if (std::regex_match(outputName, std::regex("(.*)(\\.hdr)"))) {
        camera->film->saveHDR(outputName.c_str());
    } else {
        std::cout << "Only support output as PNG/HDR\n";
    }
}
