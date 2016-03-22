// Includes
#include "globs.h"
#include "physical_constants.h"
#include "main_control_task.h"

//******************************************************************************
void MainControlTask::customMode(void)
{
    if (modes_.state == STATE_NORMAL)
    {
        // Override motion commands so robot spins in circles.
        motion_commands_.linear_velocity = 0;
        motion_commands_.angular_velocity = PI;
    }

    // Run normal balance code.  If this custom mode (or any mode) needs to
    // do balancing then the mode ID must be listed in the isVerticalConfiguration()
    // method of the modes task.
    balanceMode();
}
