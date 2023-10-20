#include <iostream>
#include <chrono>

namespace pfm {
    class FPSCounter {
    public:
        FPSCounter() : frameCount(0), fps(0) {
            lastTime = std::chrono::high_resolution_clock::now();
        }

        void update() {
            frameCount++;

            auto currentTime = std::chrono::high_resolution_clock::now();
            auto deltaTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastTime);

            if (deltaTime.count() >= 1) {
                fps = frameCount;
                frameCount = 0;
                lastTime = currentTime;
            }
        }

        int getFPS() const {
            return fps;
        }

    private:
        int frameCount;
        int fps;
        std::chrono::time_point<std::chrono::high_resolution_clock> lastTime;
    };
}
