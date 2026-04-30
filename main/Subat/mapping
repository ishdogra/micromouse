#ifndef MAPPING_H
#define MAPPING_H

#include <stdbool.h>
#include <stdint.h>

#define MAZE_SIZE 16U
#define FLOOD_FILL_INF 0xFFFFU

typedef enum
{
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
} Direction;

typedef struct
{
    uint8_t walls;       /* Bitmask for walls that are present */
    uint8_t known;       /* Bitmask for directions that have been measured */
    bool visited;
} MazeCell;

typedef struct
{
    MazeCell cells[MAZE_SIZE][MAZE_SIZE];
    uint16_t dist[MAZE_SIZE][MAZE_SIZE];
} MazeMap;

typedef struct
{
    uint8_t x;
    uint8_t y;
    Direction heading;
} MousePose;

void Mapping_Init(MazeMap *map);
void Mapping_ResetDistances(MazeMap *map);
void Mapping_UpdateCellFromSensors(MazeMap *map,
                                   const MousePose *pose,
                                   bool front_wall,
                                   bool left_wall,
                                   bool right_wall);
void Mapping_FloodFillToCenter(MazeMap *map);
void Mapping_FloodFillToCell(MazeMap *map, uint8_t goal_x, uint8_t goal_y);
Direction Mapping_ChooseNextDirection(const MazeMap *map, const MousePose *pose);
void Mapping_AdvancePose(MousePose *pose, Direction move_dir);
bool Mapping_AtCenterGoal(const MousePose *pose);
bool Mapping_CanMove(const MazeMap *map, uint8_t x, uint8_t y, Direction dir);

/* Optional debug helpers */
void Mapping_DebugPrintDistances(const MazeMap *map);
void Mapping_DebugPrintKnownWalls(const MazeMap *map);

#endif /* MAPPING_H */
