#include "mapping.h"

static MazeMap g_map;
static MousePose g_pose = {0U, 0U, DIR_NORTH};

void App_Init(void)
{
    Mapping_Init(&g_map);
}

void MappingStep(bool front_wall, bool left_wall, bool right_wall)
{
    Direction next_dir;

    /* Update current cell from sensors */
    Mapping_UpdateCellFromSensors(&g_map, &g_pose, front_wall, left_wall, right_wall);

    /* Build distance map toward the center goal */
    Mapping_FloodFillToCenter(&g_map);

    /* Pick the next direction */
    next_dir = Mapping_ChooseNextDirection(&g_map, &g_pose);

    /*
     * Send next_dir to your motion controller / state machine here.
     * Only call Mapping_AdvancePose() AFTER the robot successfully
     * completes the move into the next cell.
     */

    Mapping_AdvancePose(&g_pose, next_dir);
}
