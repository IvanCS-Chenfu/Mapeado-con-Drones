#pragma once

#include <deque>
#include <optional>
#include <utility>

namespace orbslam3_server
{

template<typename FiducialTask, typename LoopTask>
struct SecondaryTaskSelection
{
    std::optional<FiducialTask> fiducial;
    std::optional<LoopTask> loop;
};

// Se invoca solo cuando la tarea activa ha terminado. Por eso la prioridad no
// interrumpe trabajos: selecciona el FIFO fiducial y, si esta vacio, el loop.
template<typename FiducialTask, typename LoopTask>
SecondaryTaskSelection<FiducialTask, LoopTask> SelectNextSecondaryTask(
    std::deque<FiducialTask>& fiducial_queue,
    std::deque<LoopTask>& loop_queue)
{
    SecondaryTaskSelection<FiducialTask, LoopTask> selection;
    if (!fiducial_queue.empty())
    {
        selection.fiducial = std::move(fiducial_queue.front());
        fiducial_queue.pop_front();
    }
    else if (!loop_queue.empty())
    {
        selection.loop = std::move(loop_queue.front());
        loop_queue.pop_front();
    }
    return selection;
}

}  // namespace orbslam3_server
