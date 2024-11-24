// BRAND NEW HANDMADE CODE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/position_sensor.h>

#define NUM_SENSORS 8
#define NUM_LIGHT_SENSORS 8
#define RANGE 1024
#define JUNCTION_THRESHOLD 0.7
#define TIME_STEP 64
#define MAX_SPEED 6.28
#define SPEED_UNIT 0.00628
#define LIGHT_THRESHOLD 80
#define EXPLORATION_TIMEOUT 3000  // 300 seconds = 5 minutes
#define MIN_CELL_VISITS 5       // Minimum visits to consider area explored
#define GRID_SIZE 20            // Size of tracking grid
#define CELL_SIZE 0.5          // Size of each cell in virtual grid
#define POSITION_THRESHOLD 0.001    // Reduced from 1000.0 to 0.1 meters
#define WHEEL_RADIUS 0.0205      // Radius of the wheels in meters
#define AXLE_LENGTH 0.052        // Distance between wheels in meters
#define ENCODER_RESOLUTION 1000   // Encoder ticks per revolution

// Sensor weights for wall following behavior
const double sensor_weights[8][2] = {
    {150, -35},  // ps0 (front-right)
    {100, -15},  // ps1
    {80, -10},   // ps2 (right)
    {-10, -10},  // ps3
    {-10, -10},  // ps4
    {-10, 80},   // ps5 (left)
    {-30, 100},  // ps6
    {-20, 150}   // ps7 (front-left)
};

typedef struct {
    double x, y;
    double intensity;
} LightPoint;

typedef struct {
    WbDeviceTag sensors[NUM_SENSORS];
    WbDeviceTag light_sensors[NUM_LIGHT_SENSORS];
    WbDeviceTag left_motor;
    WbDeviceTag right_motor;
    WbDeviceTag left_position_sensor;
    WbDeviceTag right_position_sensor;
    double sensor_values[NUM_SENSORS];
    double light_values[NUM_LIGHT_SENSORS];
    int current_direction;  // 0: forward, 1: left, 2: back, 3: right
    LightPoint highest_light_point;
    LightPoint current_position;
    
    // Add new members for maze completion detection
    int grid[GRID_SIZE][GRID_SIZE];    // Grid to track visited areas
    double start_time;                  // Start time of exploration
    int total_junctions_found;          // Count of unique junctions found
    int revisited_junctions;            // Count of revisited junctions
    bool maze_completed;                // Flag for maze completion
    double last_junction_time;          // Time of last new junction discovery
    double last_new_area_time;          // Time of last new area discovery
     // Improved position tracking
    double prev_left_pos;
    double prev_right_pos;
    double orientation;          // Robot's orientation in radians
    
    // Light source tracking
    bool light_source_found;
    int consecutive_light_readings;
    double light_confidence;
} Robot;

// Function to convert robot position to grid coordinates
void get_grid_coordinates(double x, double y, int *grid_x, int *grid_y) {
    *grid_x = (int)((x / CELL_SIZE) + (GRID_SIZE / 2));
    *grid_y = (int)((y / CELL_SIZE) + (GRID_SIZE / 2));
    
    // Ensure coordinates are within grid bounds
    *grid_x = (*grid_x < 0) ? 0 : (*grid_x >= GRID_SIZE) ? GRID_SIZE - 1 : *grid_x;
    *grid_y = (*grid_y < 0) ? 0 : (*grid_y >= GRID_SIZE) ? GRID_SIZE - 1 : *grid_y;
}

void update_exploration_status(Robot *robot) {
    int grid_x, grid_y;
    get_grid_coordinates(robot->current_position.x, robot->current_position.y, &grid_x, &grid_y);
    
    // Update visit count for current cell
    robot->grid[grid_x][grid_y]++;
    
    // Check if this is a new area
    if (robot->grid[grid_x][grid_y] == 1) {
        robot->last_new_area_time = wb_robot_get_time();
    }
}

void init_robot(Robot *robot) {
    wb_robot_init();
    
    // Initialize motors
    robot->left_motor = wb_robot_get_device("left wheel motor");
    robot->right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(robot->left_motor, INFINITY);
    wb_motor_set_position(robot->right_motor, INFINITY);
    wb_motor_set_velocity(robot->left_motor, 0.0);
    wb_motor_set_velocity(robot->right_motor, 0.0);
    
    // Initialize distance sensors
    char sensor_name[4];
    for (int i = 0; i < NUM_SENSORS; i++) {
        sprintf(sensor_name, "ps%d", i);
        robot->sensors[i] = wb_robot_get_device(sensor_name);
        wb_distance_sensor_enable(robot->sensors[i], TIME_STEP);
    }
    
    // Initialize light sensors
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        sprintf(sensor_name, "ls%d", i);
        robot->light_sensors[i] = wb_robot_get_device(sensor_name);
        wb_light_sensor_enable(robot->light_sensors[i], TIME_STEP);
    }
    
    // Initialize position sensors
    robot->left_position_sensor = wb_robot_get_device("left wheel sensor");
    robot->right_position_sensor = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(robot->left_position_sensor, TIME_STEP);
    wb_position_sensor_enable(robot->right_position_sensor, TIME_STEP);
    
    // Initialize other variables
    robot->current_direction = 0;
    robot->highest_light_point.intensity = 0.0;
    robot->highest_light_point.x = 0.0;
    robot->highest_light_point.y = 0.0;
    robot->current_position.x = 0.0;
    robot->current_position.y = 0.0;
     
    // Initialize new members
    memset(robot->grid, 0, sizeof(robot->grid));
    robot->start_time = wb_robot_get_time();
    robot->total_junctions_found = 0;
    robot->revisited_junctions = 0;
    robot->maze_completed = false;
    robot->last_junction_time = 0;
    robot->last_new_area_time = 0;
    
    printf("Robot initialized for wall following and light seeking\n");
    
     // Initialize new tracking variables
    robot->prev_left_pos = 0.0;
    robot->prev_right_pos = 0.0;
    robot->orientation = 0.0;
    robot->light_source_found = false;
    robot->consecutive_light_readings = 0;
    robot->light_confidence = 0.0;
}


void read_sensors(Robot *robot) {
    // Read distance sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        robot->sensor_values[i] = wb_distance_sensor_get_value(robot->sensors[i]);
    }
    
    // Read light sensors
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        robot->light_values[i] = wb_light_sensor_get_value(robot->light_sensors[i]);
    }
}

void update_position(Robot *robot) {
    double left_position = wb_position_sensor_get_value(robot->left_position_sensor);
    double right_position = wb_position_sensor_get_value(robot->right_position_sensor);
    
    // Calculate wheel movements since last update
    double left_diff = left_position - robot->prev_left_pos;
    double right_diff = right_position - robot->prev_right_pos;
    
    // Update previous positions
    robot->prev_left_pos = left_position;
    robot->prev_right_pos = right_position;
    
    // Calculate distance traveled by each wheel
    double left_dist = left_diff * WHEEL_RADIUS;
    double right_dist = right_diff * WHEEL_RADIUS;
    
    // Calculate average distance and change in orientation
    double distance = (left_dist + right_dist) / 2.0;
    double d_theta = (right_dist - left_dist) / AXLE_LENGTH;
    
    // Update orientation
    robot->orientation += d_theta;
    
    // Normalize orientation to [-π, π]
    while (robot->orientation > M_PI) robot->orientation -= 2 * M_PI;
    while (robot->orientation < -M_PI) robot->orientation += 2 * M_PI;
    
    // Update position using new orientation
    robot->current_position.x += distance * cos(robot->orientation);
    robot->current_position.y += distance * sin(robot->orientation);
}

void check_light_intensity(Robot *robot) {
    double current_max_intensity = 0.0;
    double avg_intensity = 0.0;
    
    // Calculate average and find maximum intensity
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        avg_intensity += robot->light_values[i];
        if (robot->light_values[i] > current_max_intensity) {
            current_max_intensity = robot->light_values[i];
        }
    }
    avg_intensity /= NUM_LIGHT_SENSORS;
    
    // Update light source tracking with confidence
    if (current_max_intensity > robot->highest_light_point.intensity) {
        robot->consecutive_light_readings++;
        
        // Only update position if we have consistent readings
        if (robot->consecutive_light_readings >= 3) {
            robot->highest_light_point.intensity = current_max_intensity;
            robot->highest_light_point.x = robot->current_position.x;
            robot->highest_light_point.y = robot->current_position.y;
            robot->light_confidence = (current_max_intensity / LIGHT_THRESHOLD) * 
                                    (robot->consecutive_light_readings / 10.0);
            
            printf("New highest light intensity detected: %.3f at (%.3f, %.3f) with confidence %.2f%%\n",
                   current_max_intensity, robot->current_position.x, 
                   robot->current_position.y, robot->light_confidence * 100);
        }
    } else {
        robot->consecutive_light_readings = 0;
    }
}

int detect_junction(Robot *robot) {
    int left_open = 0, right_open = 0, front_open = 1;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        double normalized_value = 1.0 - (robot->sensor_values[i] / RANGE);
        
        if (normalized_value > JUNCTION_THRESHOLD) {
            if (i < NUM_SENSORS / 3) right_open = 1;
            else if (i > 2 * NUM_SENSORS / 3) left_open = 1;
            else front_open = 0;
        }
    }
    
    return (left_open + right_open + front_open) >= 2;
}

void analyze_junction(Robot *robot) {
    int left_open = 0, right_open = 0, front_open = 1;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        double normalized_value = 1.0 - (robot->sensor_values[i] / RANGE);
        
        if (normalized_value > JUNCTION_THRESHOLD) {
            if (i < NUM_SENSORS / 3) right_open = 1;
            else if (i > 2 * NUM_SENSORS / 3) left_open = 1;
            else front_open = 0;
        }
    }
    
    // Track junction discovery
    int grid_x, grid_y;
    get_grid_coordinates(robot->current_position.x, robot->current_position.y, &grid_x, &grid_y);
    
    if (robot->grid[grid_x][grid_y] == 0) {
        robot->total_junctions_found++;
        robot->last_junction_time = wb_robot_get_time();
    } else {
        robot->revisited_junctions++;
    }
    
    // Left wall following priority
    if (left_open && robot->current_direction != 3) {
        robot->current_direction = 1;  // Turn left
    } else if (front_open) {
        robot->current_direction = 0;  // Go straight
    } else if (right_open && robot->current_direction != 1) {
        robot->current_direction = 3;  // Turn right
    } else {
        robot->current_direction = 2;  // Turn back
    }
}

// Calculate angle to target (helper function)
double calculate_angle_to_target(Robot *robot, double target_x, double target_y) {
    double dx = target_x - robot->current_position.x;
    double dy = target_y - robot->current_position.y;
    return atan2(dy, dx);
}

// Unified wall-following behavior for both exploration and return phases
void calculate_speeds(Robot *robot, double *left_speed, double *right_speed, bool returning) {
    *left_speed = 0.0;
    *right_speed = 0.0;
    
    // Base wall-following behavior using sensor weights - same for all phases
    for (int i = 0; i < NUM_SENSORS; i++) {
        double normalized_value = 1.0 - (robot->sensor_values[i] / RANGE);
        *left_speed += SPEED_UNIT * sensor_weights[i][0] * normalized_value;
        *right_speed += SPEED_UNIT * sensor_weights[i][1] * normalized_value;
    }
    
    // Apply direction modifications based on current_direction
    switch (robot->current_direction) {
        case 1:  // Left turn
            *left_speed *= 0.5;
            *right_speed *= 1.5;
            break;
            
        case 2:  // Turn back
            *left_speed *= -1.0;
            *right_speed *= 1.0;
            break;
            
        case 3:  // Right turn
            *left_speed *= 1.5;
            *right_speed *= 0.5;
            break;
            
        default:  // Forward (case 0)
            // Add slight correction based on front sensors
            double front_left = robot->sensor_values[7] / RANGE;
            double front_right = robot->sensor_values[0] / RANGE;
            if (front_left > 0.3 || front_right > 0.3) {
                if (front_left > front_right) {
                    *left_speed *= 1.1;
                    *right_speed *= 0.9;
                } else {
                    *left_speed *= 0.9;
                    *right_speed *= 1.1;
                }
            }
            break;
    }
    
    // Add momentum to avoid sudden direction changes - same for all phases
    static double prev_left_speed = 0.0;
    static double prev_right_speed = 0.0;
    const double MOMENTUM = 0.3;
    
    *left_speed = (1.0 - MOMENTUM) * (*left_speed) + MOMENTUM * prev_left_speed;
    *right_speed = (1.0 - MOMENTUM) * (*right_speed) + MOMENTUM * prev_right_speed;
    
    prev_left_speed = *left_speed;
    prev_right_speed = *right_speed;
    
    // Final speed bounds check - same for all phases
    *left_speed = fmax(-MAX_SPEED, fmin(MAX_SPEED, *left_speed));
    *right_speed = fmax(-MAX_SPEED, fmin(MAX_SPEED, *right_speed));
}

bool check_maze_completion(Robot *robot) {
    double current_time = wb_robot_get_time();
    double time_since_start = current_time - robot->start_time;
    double time_since_last_junction = current_time - robot->last_junction_time;
    double time_since_last_new_area = current_time - robot->last_new_area_time;
    
    // Count well-explored cells
    int well_explored_cells = 0;
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            if (robot->grid[i][j] >= MIN_CELL_VISITS) {
                well_explored_cells++;
            }
        }
    }
    
    // Conditions for maze completion
    bool timeout_reached = time_since_start > EXPLORATION_TIMEOUT;
    bool no_new_discoveries = time_since_last_new_area > 500.0 && time_since_last_junction > 500.0;
    bool sufficient_exploration = well_explored_cells > 0 && 
                                robot->revisited_junctions > robot->total_junctions_found *1.5;
    
    if ((timeout_reached || no_new_discoveries) && sufficient_exploration) {
        if (!robot->maze_completed) {
            printf("\nMaze exploration completed!\n");
            printf("Total time: %.2f seconds\n", time_since_start);
            printf("Total junctions found: %d\n", robot->total_junctions_found);
            printf("Revisited junctions: %d\n", robot->revisited_junctions);
            printf("Well-explored cells: %d\n", well_explored_cells);
        }
        return true;
    }
    return false;
}

// Check if robot has reached target
bool check_reached_target(Robot *robot) {
    double dx = robot->highest_light_point.x - robot->current_position.x;
    double dy = robot->highest_light_point.y - robot->current_position.y;
    double distance = sqrt(dx*dx + dy*dy);
    
    // Check both position and light intensity
    bool position_reached = distance < POSITION_THRESHOLD;
    bool light_detected = false;
    
    // Check if we're actually at a bright spot
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        if (robot->light_values[i] > robot->highest_light_point.intensity * 0.9) {
            light_detected = true;
            break;
        }
    }
    
    return position_reached && light_detected;
}

int main() {
    Robot robot;
    init_robot(&robot);
    
    bool exploration_complete = false;
    bool reached_target = false;
    
    while (wb_robot_step(TIME_STEP) != -1) {
        read_sensors(&robot);
        update_position(&robot);
        
        if (!exploration_complete) {
            update_exploration_status(&robot);
            check_light_intensity(&robot);
            
            // Check for maze completion
            if (check_maze_completion(&robot)) {
                exploration_complete = true;
                printf("Starting return to highest light intensity point (%.3f, %.3f)...\n",
                       robot.highest_light_point.x,
                       robot.highest_light_point.y);
                continue;
            }
            
            // Regular exploration behavior
            if (detect_junction(&robot)) {
                analyze_junction(&robot);
            }
            
            double left_speed, right_speed;
            calculate_speeds(&robot, &left_speed, &right_speed, false);
            wb_motor_set_velocity(robot.left_motor, left_speed);
            wb_motor_set_velocity(robot.right_motor, right_speed);
        } else {
            // Return navigation mode using hybrid approach
            if (!reached_target) {
                reached_target = check_reached_target(&robot);
                if (reached_target) {
                    printf("Reached highest light intensity point!\n");
                    wb_motor_set_velocity(robot.left_motor, 0.0);
                    wb_motor_set_velocity(robot.right_motor, 0.0);
                    break;
                }
                
                double left_speed, right_speed;
                calculate_speeds(&robot, &left_speed, &right_speed, true);
                wb_motor_set_velocity(robot.left_motor, left_speed);
                wb_motor_set_velocity(robot.right_motor, right_speed);
            }
        }
    }
    
    // Final report
    printf("\nExploration Summary:\n");
    printf("Highest light intensity found: %.2f at position (%.3f, %.3f)\n",
           robot.highest_light_point.intensity,
           robot.highest_light_point.x,
           robot.highest_light_point.y);
    printf("Final position: (%.3f, %.3f)\n",
           robot.current_position.x,
           robot.current_position.y);
    
    wb_robot_cleanup();
    return 0;
}