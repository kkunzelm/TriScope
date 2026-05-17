// Standalone VIFF I/O test: read .xv files, print stats, round-trip test.
// Build target: viff_test  (not part of the main GUI application)
#include "ViffReader.h"
#include "ViffWriter.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

static void printStats(const char* label, const ViffImage& img) {
    std::printf("=== %s ===\n", label);
    std::printf("  Dimensions : %u rows x %u cols\n", img.rows, img.cols);
    std::printf("  Pixel size : %.4f mm x %.4f mm\n",
                img.xPixelSize * 1000.0f, img.yPixelSize * 1000.0f);
    std::printf("  Origin     : (%.4f, %.4f)\n", img.originX, img.originY);

    float zMin = std::numeric_limits<float>::max();
    float zMax = std::numeric_limits<float>::lowest();
    double sum = 0.0;
    uint32_t valid = 0;

    for (uint32_t r = 0; r < img.rows; ++r) {
        for (uint32_t c = 0; c < img.cols; ++c) {
            if (img.isValid(r, c)) {
                float v = img.at(r, c);
                zMin = std::min(zMin, v);
                zMax = std::max(zMax, v);
                sum += v;
                ++valid;
            }
        }
    }

    uint32_t total = img.totalPixels();
    std::printf("  Total px   : %u\n", total);
    std::printf("  Valid px   : %u  (%.1f%%)\n",
                valid, 100.0f * valid / static_cast<float>(total));
    if (valid > 0) {
        std::printf("  Z min/max  : %.2f / %.2f\n", zMin, zMax);
        std::printf("  Z mean     : %.2f\n", sum / valid);
    }
    std::printf("\n");
}

static bool roundTripTest(const ViffImage& original, const char* tmpPath) {
    ViffWriter writer;
    if (!writer.save(tmpPath, original)) {
        std::fprintf(stderr, "Round-trip WRITE failed: %s\n", writer.lastError().c_str());
        return false;
    }

    ViffImage reloaded;
    ViffReader reader;
    if (!reader.load(tmpPath, reloaded)) {
        std::fprintf(stderr, "Round-trip READ failed: %s\n", reader.lastError().c_str());
        return false;
    }

    bool ok = true;

    if (reloaded.rows != original.rows || reloaded.cols != original.cols) {
        std::fprintf(stderr, "Round-trip FAIL: dimensions differ (%u x %u vs %u x %u)\n",
                     reloaded.rows, reloaded.cols, original.rows, original.cols);
        ok = false;
    }

    if (reloaded.data.size() != original.data.size()) {
        std::fprintf(stderr, "Round-trip FAIL: data size differs\n");
        ok = false;
    }

    if (ok) {
        uint32_t diffs = 0;
        for (size_t i = 0; i < original.data.size(); ++i) {
            float a = original.data[i], b = reloaded.data[i];
            // NaN-aware comparison
            bool bothNan = (std::isnan(a) && std::isnan(b));
            if (!bothNan && a != b) ++diffs;
        }
        if (diffs > 0) {
            std::fprintf(stderr, "Round-trip FAIL: %u pixel values differ\n", diffs);
            ok = false;
        }
    }

    std::remove(tmpPath);

    if (ok) std::printf("Round-trip OK (write+read, bit-identical)\n\n");
    return ok;
}

int main(int argc, char* argv[]) {
    const char* testFiles[] = {
        "data/3d-data/1_17_16_I.xv",
        "data/3d-data/1_17_16_II.xv",
        "data/3d-data/27_46_47_I.xv",
        "data/3d-data/27_46_47_II.xv",
    };
    // Allow overriding with CLI arguments
    std::vector<std::string> files;
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) files.emplace_back(argv[i]);
    } else {
        for (auto* f : testFiles) files.emplace_back(f);
    }

    bool allOk = true;
    ViffReader reader;

    for (const auto& path : files) {
        ViffImage img;
        if (!reader.load(path, img)) {
            std::fprintf(stderr, "LOAD FAILED: %s  -- %s\n\n",
                         path.c_str(), reader.lastError().c_str());
            allOk = false;
            continue;
        }
        printStats(path.c_str(), img);
        if (!roundTripTest(img, "/tmp/viff_roundtrip_test.xv")) {
            allOk = false;
        }
    }

    return allOk ? 0 : 1;
}
