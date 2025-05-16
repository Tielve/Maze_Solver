#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/random/random.h>

#define MAZE_WIDTH  5
#define MAZE_HEIGHT 5
#define ACTION_COUNT 4
#define STATE_COUNT (MAZE_WIDTH * MAZE_HEIGHT)
#define STACK_SIZE 1024
#define SENSOR_PRIORITY   3
#define AI_PRIORITY       2
#define ACTUATOR_PRIORITY 3

void sensor_thread(void *a, void *b, void *c);
void ai_thread(void *a, void *b, void *c);
void actuator_thread(void *a, void *b, void *c);

K_THREAD_DEFINE(sensor_tid, STACK_SIZE, sensor_thread, NULL, NULL, NULL, SENSOR_PRIORITY, 0, 0);
K_THREAD_DEFINE(ai_tid, STACK_SIZE, ai_thread,     NULL, NULL, NULL, AI_PRIORITY,     0, 0);
K_THREAD_DEFINE(actuator_tid, STACK_SIZE, actuator_thread, NULL, NULL, NULL, ACTUATOR_PRIORITY, 0, 0);

typedef struct {
    int x;
    int y;
} position_t;

typedef enum {
    MOVE_UP,
    MOVE_DOWN,
    MOVE_LEFT,
    MOVE_RIGHT,
} action_t;

//Simulates sensor data
typedef struct {
    bool wall_up;
    bool wall_down;
    bool wall_left;
    bool wall_right;
} sensor_data_t;

typedef struct {
    action_t action;
} action_cmd_t;

//Create 2D maze, start at (0,0) and goal is (5,5) 1 is walls 0 is open space
static const uint8_t maze[MAZE_HEIGHT][MAZE_WIDTH] = {
    {0, 1, 0, 0, 0},
    {0, 1, 0, 1, 0},
    {0, 0, 0, 1, 0},
    {1, 1, 0, 1, 0},
    {0, 0, 0, 0, 0}
};

K_MSGQ_DEFINE(sensor_queue, sizeof(sensor_data_t), 5, 4);
K_MSGQ_DEFINE(action_queue, sizeof(action_cmd_t), 5, 4);

static position_t agent_pos = {0, 0};
static const position_t goal_pos = {4, 4};
float q_table[STATE_COUNT][ACTION_COUNT] = {0};

//Simulates a sensor checking its 4 walls around it,
//if it is on the boundary return a 1, simulating a physical boundary
void sensor_thread(void *a, void *b, void *c) {
    sensor_data_t sensors;

    while (1) {
        if(agent_pos.y == MAZE_HEIGHT-1 && agent_pos.x == MAZE_WIDTH-1) {
            agent_pos.y = 0;
            agent_pos.x = 0;
        }
        if(agent_pos.y != 0){
            sensors.wall_up = maze[agent_pos.y - 1][agent_pos.x];
        }
        else {
            sensors.wall_up = 1;
        }
        if(agent_pos.y != MAZE_HEIGHT - 1) {
            sensors.wall_down = maze[agent_pos.y + 1][agent_pos.x];
        }
        else {
            sensors.wall_down = 1;
        }

        if(agent_pos.x != MAZE_WIDTH - 1){
            sensors.wall_right = maze[agent_pos.y][agent_pos.x + 1];
        }
        else {
            sensors.wall_right = 1;
        }

        if(agent_pos.x != 0){
            sensors.wall_left = maze[agent_pos.y][agent_pos.x - 1];
        }
        else {
            sensors.wall_left = 1;
        }

        k_msgq_put(&sensor_queue, &sensors, K_FOREVER);
        k_msleep(100); //Simulate sensor rate
    }
}

int get_state_index(position_t pos) {
    return pos.y * MAZE_WIDTH + pos.x;
}


void ai_thread(void *a, void *b, void *c) {
    sensor_data_t sensors;
    action_cmd_t cmd;
    position_t new_pos;

    while (1) {
        k_msgq_get(&sensor_queue, &sensors, K_FOREVER);

        //Randomize actions and restart when at goal
        do {
            cmd.action = sys_rand32_get() % ACTION_COUNT;

            new_pos = agent_pos;
            if (cmd.action == MOVE_UP) {
                new_pos.y--;
            }
            if (cmd.action == MOVE_DOWN) {
                new_pos.y++;
            }
            if (cmd.action == MOVE_LEFT) {
                new_pos.x--;
            }
            if (cmd.action == MOVE_RIGHT) {
                new_pos.x++;
            }
        } while (
            new_pos.x < 0 || new_pos.x >= MAZE_WIDTH ||
            new_pos.y < 0 || new_pos.y >= MAZE_HEIGHT ||
            maze[new_pos.y][new_pos.x] == 1
        );

        k_msgq_put(&action_queue, &cmd, K_FOREVER);
    }
}

void actuator_thread(void *a, void *b, void *c) {
    action_cmd_t cmd;
    while (1) {
        k_msgq_get(&action_queue, &cmd, K_FOREVER);

        if (cmd.action == MOVE_UP && agent_pos.y > 0 && !maze[agent_pos.y - 1][agent_pos.x])
            agent_pos.y--;
        if (cmd.action == MOVE_DOWN && agent_pos.y < MAZE_HEIGHT - 1 && !maze[agent_pos.y + 1][agent_pos.x])
            agent_pos.y++;
        if (cmd.action == MOVE_LEFT && agent_pos.x > 0 && !maze[agent_pos.y][agent_pos.x - 1])
            agent_pos.x--;
        if (cmd.action == MOVE_RIGHT && agent_pos.x < MAZE_WIDTH - 1 && !maze[agent_pos.y][agent_pos.x + 1])
            agent_pos.x++;

        printk("Agent position: (%d, %d)\n", agent_pos.x, agent_pos.y);
    }
}

int main(void)
{
    printk("Maze AI starting...\n");
    return 0;
}
