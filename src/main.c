#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/random/random.h>

#define MAZE_WIDTH  5
#define MAZE_HEIGHT 5
#define ACTION_COUNT 4
#define STATE_COUNT (MAZE_WIDTH * MAZE_HEIGHT)
#define STACK_SIZE 1024
#define SENSOR_PRIORITY   2
#define AI_PRIORITY       1
#define ACTUATOR_PRIORITY 3
#define EPSILON 0.1f
#define GAMMA 0.9f
#define ALPHA 0.1f


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

action_t choose_action(position_t pos) {
    int state = get_state_index(pos);

    //When less than epsilon explore, get random action
    if ((float)sys_rand32_get() / UINT32_MAX < EPSILON) {
        return (action_t)(sys_rand32_get() % ACTION_COUNT);
    }

    //Make max_q a temp arbitrarily low number, and make best move an arbitrary action
    float max_q = -9999.0f;
    action_t best = MOVE_UP;

    //Find best action based on Q table
    for (int i = 0; i < ACTION_COUNT; i++) {
        if (q_table[state][i] > max_q) {
            max_q = q_table[state][i];
            best = (action_t)i;
        }
    }
    return best;
}

void update_q(position_t old_pos, action_t action, position_t new_pos, float reward) {
    int s = get_state_index(old_pos);
    int s_prime = get_state_index(new_pos);

    //Get first Max Q to compare with in for loop
    float max_q_next = q_table[s_prime][0];
    for (int i = 1; i < ACTION_COUNT; i++) {
        if (q_table[s_prime][i] > max_q_next) {
            max_q_next = q_table[s_prime][i];
        }
    }

    //Q-learning update formula
    q_table[s][action] += ALPHA * (reward + GAMMA * max_q_next - q_table[s][action]);
}

position_t get_resulting_pos(position_t pos, action_cmd_t cmd) {
    position_t result = pos;
    if (cmd.action == MOVE_UP && agent_pos.y > 0 && !maze[agent_pos.y - 1][agent_pos.x]) {
        result.y--;
    }
    if (cmd.action == MOVE_DOWN && agent_pos.y < MAZE_HEIGHT - 1 && !maze[agent_pos.y + 1][agent_pos.x]) {
        result.y++;
    }
    if (cmd.action == MOVE_LEFT && agent_pos.x > 0 && !maze[agent_pos.y][agent_pos.x - 1]) {
        result.x--;
    }
    if (cmd.action == MOVE_RIGHT && agent_pos.x < MAZE_WIDTH - 1 && !maze[agent_pos.y][agent_pos.x + 1]) {
        result.x++;
    }
    return result;
}

float get_reward(position_t old_pos, position_t new_pos) {
    //Reached goal
    if (new_pos.x == goal_pos.x && new_pos.y == goal_pos.y) {
        return 100.0f;
    }

    //Invalid move
    if (new_pos.x == old_pos.x && new_pos.y == old_pos.y) {
        return -10.0f;
    }

    //Any other valid move
    return -1.0f;
}

void ai_thread(void *a, void *b, void *c) {
    sensor_data_t sensors;
    action_cmd_t cmd;
    position_t new_pos;
    position_t old_pos;
    float reward;

    while (1) {
        k_msgq_get(&sensor_queue, &sensors, K_FOREVER);
        old_pos = agent_pos;
        cmd.action = choose_action(old_pos);

        new_pos = get_resulting_pos(old_pos, cmd);
        reward = get_reward(old_pos, new_pos);
        update_q(old_pos, cmd.action, new_pos, reward);

        if (reward != -10) {
            agent_pos = new_pos;
        }

        k_msgq_put(&action_queue, &cmd, K_FOREVER);

        if (agent_pos.x == MAZE_WIDTH - 1 && agent_pos.y == MAZE_HEIGHT - 1) {
            agent_pos.x = 0;
            agent_pos.y = 0;
        }

        k_msleep(100);
    }
}

void actuator_thread(void *a, void *b, void *c) {
    action_cmd_t cmd;
    while (1) {
        k_msgq_get(&action_queue, &cmd, K_FOREVER);
        printk("Agent position: (%d, %d)\n", agent_pos.x, agent_pos.y);
    }
}

int main(void)
{
    printk("Maze AI starting...\n");
    return 0;
}
