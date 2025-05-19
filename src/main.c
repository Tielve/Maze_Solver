#include <esp_heap_caps.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/random/random.h>
#include "esp_psram.h"
#include "zephyr/drivers/spi.h"
#include "zephyr/multi_heap/shared_multi_heap.h"

#define MAZE_WIDTH  239
#define MAZE_HEIGHT 339
#define ACTION_COUNT 4
#define STATE_COUNT (MAZE_WIDTH * MAZE_HEIGHT)
#define STACK_SIZE 4096
#define SENSOR_PRIORITY   3
#define AI_PRIORITY       2
#define ACTUATOR_PRIORITY 4
#define EPSILON 0.1f
#define GAMMA 0.9f
#define ALPHA 0.1f

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

const int dirs[4][2] = {
    { 0, -2}, // up
    { 0,  2}, // down
    {-2,  0}, // left
    { 2,  0}  // right
};
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

static __attribute__((section(".ext_ram.bss")))
uint8_t maze[MAZE_HEIGHT][MAZE_WIDTH];

static __attribute__((section(".ext_ram.bss")))
float q_table[STATE_COUNT][ACTION_COUNT];

static position_t agent_pos = {1, 1};
static const position_t goal_pos = {MAZE_HEIGHT - 2, MAZE_WIDTH - 2};
static position_t *stack = NULL;
static int top = 0;
struct k_sem maze_ready_sem;

void sensor_thread(void *a, void *b, void *c);
void ai_thread(void *a, void *b, void *c);
void actuator_thread(void *a, void *b, void *c);

K_THREAD_DEFINE(sensor_tid, STACK_SIZE, sensor_thread, NULL, NULL, NULL, SENSOR_PRIORITY, 0, 0);
K_THREAD_DEFINE(ai_tid, STACK_SIZE, ai_thread, NULL, NULL, NULL, AI_PRIORITY, 0, 0);
K_THREAD_DEFINE(actuator_tid, STACK_SIZE, actuator_thread, NULL, NULL, NULL, ACTUATOR_PRIORITY, 0, 0);
K_MSGQ_DEFINE(sensor_queue, sizeof(sensor_data_t), 5, 4);
K_MSGQ_DEFINE(action_queue, sizeof(action_cmd_t), 5, 4);

void push(position_t c) {
    stack[top++] = c;
}

position_t pop(void) {
    return stack[--top];
}

bool stack_empty(void) {
    return top == 0;
}

void shuffle(int *array, int n) {
    for (int i = n - 1; i > 0; i--) {
        int j = sys_rand32_get() % (i + 1);
        int temp = array[i];
        array[i] = array[j];
        array[j] = temp;
    }
}

void dfs_carve(int x, int y) {
    stack = shared_multi_heap_aligned_alloc(SMH_REG_ATTR_EXTERNAL, 4, sizeof(position_t) * STATE_COUNT);
    if (!stack) {
        printk("Error: Memory not allocated for stack");
        return;
    }

    push((position_t){x, y});
    maze[y][x] = 0;

    //Directions for DFS, move in 2 to leave walls
    const int dirs[4][2] = {
        { 0, -2}, { 0, 2},
        {-2, 0}, { 2, 0}
    };

    while (!stack_empty()) {
        position_t current = pop();

        int order[4] = {0, 1, 2, 3};
        shuffle(order, 4);

        for (int i = 0; i < 4; i++) {
            int dx = dirs[order[i]][0];
            int dy = dirs[order[i]][1];
            int nx = current.x + dx;
            int ny = current.y + dy;

            if (nx > 0 && nx < MAZE_WIDTH - 1 && ny > 0 && ny < MAZE_HEIGHT - 1 && maze[ny][nx] == 1) {
                maze[ny][nx] = 0;
                maze[current.y + dy/2][current.x + dx/2] = 0;
                push(current);  //backtrack later
                push((position_t){nx, ny});
                break;
            }
        }
    }
    free(stack);
}

//Simulates a sensor checking its 4 walls around it,
//if it is on the boundary return a 1, simulating a physical boundary
void sensor_thread(void *a, void *b, void *c) {
    k_sem_take(&maze_ready_sem, K_FOREVER);  // will block
    k_sem_give(&maze_ready_sem);
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
    k_sem_take(&maze_ready_sem, K_FOREVER);  // will block
    k_sem_give(&maze_ready_sem);
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

        if (agent_pos.x == MAZE_WIDTH - 2 && agent_pos.y == MAZE_HEIGHT - 2) {
            agent_pos.x = 1;
            agent_pos.y = 1;
        }

        k_msleep(100);
    }
}

void actuator_thread(void *a, void *b, void *c) {
    k_sem_take(&maze_ready_sem, K_FOREVER);  // will block
    k_sem_give(&maze_ready_sem);
    action_cmd_t cmd;
    while (1) {
        k_msgq_get(&action_queue, &cmd, K_FOREVER);
        printk("Agent position: (%d, %d)\n", agent_pos.x, agent_pos.y);
    }
}

int main(void)
{
    k_sem_init(&maze_ready_sem, 0, 1);
    memset(maze, 1, sizeof(maze));
    memset(q_table, 0, sizeof(q_table));

    dfs_carve(1,1);

    k_sem_give(&maze_ready_sem);

    printk("Maze AI starting...\n");

    return 0;
}
