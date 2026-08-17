#include "orbslam3_server/secondary_task_order.hpp"

#include <deque>
#include <iostream>
#include <string>
#include <vector>

int main()
{
    std::deque<std::string> fiducials;
    std::deque<std::string> loops{"loop_10", "loop_11", "loop_12"};
    std::vector<std::string> executed;

    // loop_10 ya estaba activo cuando llega fid_3: no se llama al selector
    // hasta que termina, por lo que no puede ser interrumpido.
    executed.push_back(std::move(loops.front()));
    loops.pop_front();
    fiducials.push_back("fid_3");

    while (!fiducials.empty() || !loops.empty())
    {
        auto selected = orbslam3_server::SelectNextSecondaryTask(
            fiducials, loops);
        if (selected.fiducial)
        {
            executed.push_back(std::move(selected.fiducial.value()));
        }
        else if (selected.loop)
        {
            executed.push_back(std::move(selected.loop.value()));
        }
    }

    const std::vector<std::string> expected{
        "loop_10", "fid_3", "loop_11", "loop_12"};
    if (executed != expected)
    {
        std::cerr << "[test_secondary_task_order] orden incorrecto\n";
        return 1;
    }
    std::cout << "[test_secondary_task_order] PASS\n";
    return 0;
}
