#include "collect_ui.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

// Simulates a fatal error after `delay_ms` milliseconds — lets you test the
// Cancelled display path without any hardware.
static void simulate_error_after(int delay_ms, std::atomic<int>& collection_id,
                                 std::atomic<bool>& running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    if (running.load()) {
        ui::cancel_current(collection_id.load());
        running.store(false);
    }
}

int main(int argc, char* argv[]) {
    // Pass "--error <ms>" to simulate a fatal error after that many milliseconds.
    // e.g.: ./collect_cli_test --error 5000
    int error_after_ms = -1;
    for (int i = 1; i < argc - 1; ++i) {
        if (std::string(argv[i]) == "--error")
            error_after_ms = std::stoi(argv[i + 1]);
    }

    std::atomic<bool> running{true};
    std::atomic<bool> _paused{false};
    std::atomic<int>  collection_id{1};

    std::cout << "  [collect_cli test — no hardware]"
              << "   space → pause/resume   q → stop";
    if (error_after_ms > 0)
        std::cout << "   simulated error in " << error_after_ms << "ms";
    std::cout << "\n";

    ui::add_collection(collection_id.load());

    std::thread display_thread([&] {
        while (running.load()) {
            ui::redraw();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        ui::redraw();
    });

    std::thread error_thread;
    if (error_after_ms > 0)
        error_thread = std::thread(simulate_error_after, error_after_ms,
                                   std::ref(collection_id), std::ref(running));

    {
        RawMode raw;
        while (running.load()) {
            char key = 0;
            if (read(STDIN_FILENO, &key, 1) <= 0)
                continue;

            if (key == 'q' || key == 'Q' || key == 3 /* Ctrl+C */) {
                ui::complete_current(collection_id.load());
                running.store(false);
            } else if (key == ' ') {
                if (!_paused.load()) {
                    ui::complete_current(collection_id.load());
                    collection_id.fetch_add(1);
                    ui::add_next(collection_id.load());
                    _paused.store(true);
                } else {
                    if (ui::start_next_collection())
                        _paused.store(false);
                }
            }
        }
    }

    display_thread.join();
    if (error_thread.joinable())
        error_thread.join();

    return 0;
}
