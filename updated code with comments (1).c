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
#include <webots/led.h>  // Add LED 
#include <webots/speaker.h>  // Add speaker 

#define NUM_SENSORS 8 //no: of distance sensors on the robot
#define NUM_LIGHT_SENSORS 8//no: of light sensors on the robot
#define NUM_LEDS 8  //no: of LEDS on the robot
#define RANGE 1024 //maximum sensor reading range
#define JUNCTION_THRESHOLD 0.7 
#define TIME_STEP 64
#define MAX_SPEED 6.28   
#define SPEED_UNIT 0.00628
#define LIGHT_THRESHOLD 1100 //threshold for light detection
#define EXPLORATION_TIMEOUT 3000 // max exploration time (in seconds)
#define MIN_CELL_VISITS 5       // Minimum visits to consider area explored
#define GRID_SIZE 20            // Size of tracking grid
#define CELL_SIZE 0.5          // Size of each cell in virtual grid
#define WHEEL_RADIUS 0.0205      // Radius of the wheels in meters
#define AXLE_LENGTH 0.052        // axle lenght in meters
#define CONSECUTIVE_READINGS_REQUIRED 1 // Required consecutive light readings
#define MIN_CONFIDENCE_THRESHOLD 0.4  //minimum confidence threshold for light detection

// Sensor weights for the wall following behavior
const double sensor_weights[8][2] = {
    {150, -35},  
    {100, -15},  
    {80, -10},   
    {-10, -10},  
    {-10, -10},  
    {-10, 80},   
    {-30, 100},  
    {-20, 150}   
};

typedef struct {  //structure for wall following behaviour
    double x, y; 
    double intensity;
} light_p;

typedef struct {  //main robot structure with all necessary information
    WbDeviceTag sensors[NUM_SENSORS];  //array of 8 distance sensors on the robot
    WbDeviceTag light_sensors[NUM_LIGHT_SENSORS]; //array of 8 light sensors on the robot
    WbDeviceTag left_motor;    // control of left motor
    WbDeviceTag right_motor;   // control of right motor
    WbDeviceTag left_position_sensor;          
    WbDeviceTag right_position_sensor;
    double sensor_values[NUM_SENSORS]; //raw distance readings from sensors
    double light_values[NUM_LIGHT_SENSORS];  //raw light readings from light sensors
    int current_direction;  // 0: forward, 1: left, 2: back, 3: right
    light_p lowest_light_point; //position of weakest detected light 
    light_p current_position; // robots current postion
    
    // Add new members for maze completion detection
    int grid[GRID_SIZE][GRID_SIZE];    // Grid to track visited areas
    double start_time;                  // Start time of exploration
    int total_junctions_found;          // Count of unique junctions found
    int revisited_junctions;            // Count of revisited junctions
    bool maze_completed;                // For maze completion
    double last_junction_time;          // Time of last new junction discovery
    double last_new_area_time;          // Time of last new area discovery
     
    double prev_left_pos;   //previous left wheel postioning (used for calcualating movement)
    double prev_right_pos;  //previous right wheel postioning (used for calcualating movement)
    double orientation;          // Robot's orientation in radians
    
    // Light source tracking
    bool light_source_found;  //flag indicating a signinficant light source has been found
    int consecutive_light_readings;  //counter for consistent light readings
    double light_confidence;   //confidence level based on current readings
    
    light_p high_light_point;  // Add this new member
    WbDeviceTag leds[NUM_LEDS];  // Add LED array to the Robot struct
    
} Robot;

// Function to initialize all the LEDs on the robot
void init_leds(Robot *robot) {
    char led_name[4];
    for (int i = 0; i < NUM_LEDS; i++) {
        sprintf(led_name, "led%d", i);
        robot->leds[i] = wb_robot_get_device(led_name);
    }
}

void play_success_pattern(Robot *robot) {
    //Turn all LEDs on
    for (int i = 0; i < NUM_LEDS; i++) {
        wb_led_set(robot->leds[i], 1); 
    }
    
    // Get the speaker device
    WbDeviceTag speaker = wb_robot_get_device("speaker");
    
    // Play success sound
    if (speaker != 0) {
        // Full signature: left speaker, right speaker, sound file, volume, pitch, balance
        wb_speaker_play_sound(speaker, speaker, "C:\\Users\\ABUBAKR SALIH\\Desktop\\success.wav", 1.0, 1.0, 0.0, 0.0);
         printf("\U0001F3B5 Melody being played!!\U0001F3B5\n");
         printf("\U0001F4A1 LED's turned on!!\U0001F4A1\n");

    } else {
        printf("Speaker device not found!\n");
    }
}

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
    
    // Update visit count for the current cell
    robot->grid[grid_x][grid_y]++;
    
    // update the timestap if this is the first visit to this cell
    if (robot->grid[grid_x][grid_y] == 1) {
        robot->last_new_area_time = wb_robot_get_time();
    }
}

void init_robot(Robot *robot) {  //Initialize all robot components and variables
    wb_robot_init();
    
    // Initialize motors and set initial states
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
    robot->lowest_light_point.intensity = 0.0;
    robot->lowest_light_point.x = 0.0;
    robot->lowest_light_point.y = 0.0;
    robot->current_position.x = 0.0;
    robot->current_position.y = 0.0;
    robot->high_light_point.intensity = INFINITY;  // Initialize to infinity
    robot->high_light_point.x = 0.0;
    robot->high_light_point.y = 0.0;
    
    // Initialize new members
    memset(robot->grid, 0, sizeof(robot->grid)); //clear exploration grid
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
    
    // Initialize LEDs
    init_leds(robot);
    
    
   // Initialize speaker 
    WbDeviceTag speaker = wb_robot_get_device("speaker");
    if (speaker != 0) {
        wb_speaker_set_engine(speaker, "native");  // Use string "native"
    }
}

void read_sensors(Robot *robot) {  //read and update all the sensor values
    // Read distance sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        robot->sensor_values[i] = wb_distance_sensor_get_value(robot->sensors[i]);
    }
    
    // update light sensor readings
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        robot->light_values[i] = wb_light_sensor_get_value(robot->light_sensors[i]);
    }
}

void update_position(Robot *robot) {  //updating robots position
    double left_position = wb_position_sensor_get_value(robot->left_position_sensor);  // get current wheel positions from position sensors
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
    double sensor_intensities[NUM_LIGHT_SENSORS]; // array to store intensity reading from each sensor
    double total_intensity = 0.0;  //variable to track avg light intensity
    double avg_intensity = 0.0;    //variable to track avg light intensity
    double directional_confidence = 0.0;

    // Calculate the total intensity and average intensity across all sensors
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        sensor_intensities[i] = robot->light_values[i];
        total_intensity += sensor_intensities[i];
    }

    avg_intensity = total_intensity / NUM_LIGHT_SENSORS;

    //Display the average light intensity
    printf("Detected light intensity: %.3f at (%.3f, %.3f)\n", avg_intensity, robot->current_position.x, robot->current_position.y);

    // Calculate directional confidence based on the average light intensity
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        int sector_difference = abs(i - 0);  // Simplified: assuming center sensor is the reference
        directional_confidence += (sector_difference < 4) ? sensor_intensities[i] : 0;
    }

    // Calculate light confidence
    robot->light_confidence = (avg_intensity / (LIGHT_THRESHOLD * 2)) *
                              (directional_confidence / (total_intensity + 1));

    // Process the light detection as usual
    if (robot->light_confidence > MIN_CONFIDENCE_THRESHOLD) {
        robot->consecutive_light_readings++;

        if (robot->consecutive_light_readings >= CONSECUTIVE_READINGS_REQUIRED) {
            // Update lowest point 
            if (avg_intensity > robot->lowest_light_point.intensity * 0.5) {
                robot->lowest_light_point.intensity = avg_intensity;
                robot->lowest_light_point.x = robot->current_position.x;
                robot->lowest_light_point.y = robot->current_position.y;
            }

            // Update highest light point
            if (robot->high_light_point.intensity == INFINITY ||
                avg_intensity < robot->high_light_point.intensity) {
                robot->high_light_point.intensity = avg_intensity;
                robot->high_light_point.x = robot->current_position.x;
                robot->high_light_point.y = robot->current_position.y;

                printf("Updated Highest intensity light: %.3f at (%.3f, %.3f)\n",
                       avg_intensity, robot->current_position.x,
                       robot->current_position.y);
            }
        }
    }
}

int check_junction(Robot *robot) {
    int left_open = 0, right_open = 0, front_open = 1; //flags to track which directions are open
    
    for (int i = 0; i < NUM_SENSORS; i++) {     //check each sensors readings
        double normalized_value = 1.0 - (robot->sensor_values[i] / RANGE);
        
        if (normalized_value > JUNCTION_THRESHOLD) {  //determine which direction is open based on sensor position and reading
            if (i < NUM_SENSORS / 3) right_open = 1;  //right side sensors
            else if (i > 2 * NUM_SENSORS / 3) left_open = 1;  //left side sensors
            else front_open = 0;   //front sensors
        }
    }
    
    return (left_open + right_open + front_open) >= 2; //returns true if atleast 2 directiosn are open
}

void examine_junction(Robot *robot) {   
    int left_open = 0, right_open = 0, front_open = 1;  //flags to track which directions are open
    
    for (int i = 0; i < NUM_SENSORS; i++) {   //analyze sensor readings to determine open directions
        double normalized_value = 1.0 - (robot->sensor_values[i] / RANGE);
        
        if (normalized_value > JUNCTION_THRESHOLD) {
            if (i < NUM_SENSORS / 3) right_open = 1;
            else if (i > 2 * NUM_SENSORS / 3) left_open = 1;
            else front_open = 0;
        }
    }
    
    // get current postion and grid coordinates
    int grid_x, grid_y;
    get_grid_coordinates(robot->current_position.x, robot->current_position.y, &grid_x, &grid_y);
    
    if (robot->grid[grid_x][grid_y] == 0) {  //update junction statistics 
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

// wall following behavior for both exploration and return phases
void update_motor_controls(Robot *robot, double *left_speed, double *right_speed, bool returning) {
    *left_speed = 0.0;  // initialize speeds to zero
    *right_speed = 0.0; // initialize speeds to zero
    
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
    
    // Apply momentum to smooth out speed changes
    static double prev_left_speed = 0.0;
    static double prev_right_speed = 0.0;
    const double MOMENTUM = 0.3;
    
    *left_speed = (1.0 - MOMENTUM) * (*left_speed) + MOMENTUM * prev_left_speed;
    *right_speed = (1.0 - MOMENTUM) * (*right_speed) + MOMENTUM * prev_right_speed;
    
    prev_left_speed = *left_speed;
    prev_right_speed = *right_speed;
    
    // ensure speeds stay within bounds
    *left_speed = fmax(-MAX_SPEED, fmin(MAX_SPEED, *left_speed));
    *right_speed = fmax(-MAX_SPEED, fmin(MAX_SPEED, *right_speed));
}

bool check_maze_completion(Robot *robot) {    //function to check if the maze has been completly explored
    double current_time = wb_robot_get_time();   // calc vaious time intervals
    double time_since_start = current_time - robot->start_time;
    double time_since_last_junction = current_time - robot->last_junction_time;
    double time_since_last_new_area = current_time - robot->last_new_area_time;
    
    // Count well explored cells
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
    bool no_new_discoveries = time_since_last_new_area > 300.0 && time_since_last_junction > 300.0;
    bool sufficient_exploration = well_explored_cells > 0 && 
                                robot->revisited_junctions > robot->total_junctions_found *1.5;
    
    if ((timeout_reached || no_new_discoveries) && sufficient_exploration) {  //if condition are met , return value and print statement
        if (!robot->maze_completed) {
            printf("\nThe Maze exploration has been completed!!\n");
            printf("Total time taken to explore the maze: %.2f seconds\n", time_since_start);
            printf("Revisited junctions: %d\n", robot->revisited_junctions);
            printf("Well-explored cells: %d\n", well_explored_cells);
        }
        return true;
    }
    return false;
}

bool check_reached_highest_light(Robot *robot) {   // Check if robot has reached target
    // Calculate a threshold for original highest light intensity
    double intensity_threshold = robot->high_light_point.intensity * 1.0;  // 10% tolerance
    
    // Check if current light readings are close to the original highest light
    int matching_sensors = 0;
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        if (robot->light_values[i] <= intensity_threshold) {
            matching_sensors++;
        }
    }
    
    // Require at least half the sensors to detect similar hight intensity light
    bool intensity_match = matching_sensors >= (NUM_LIGHT_SENSORS / 2);
    
    // Additional stability check
    bool stability_check = robot->consecutive_light_readings > 3;
    
    return intensity_match && stability_check;  //return true only if both conditions above are met
}

int main() {   //Main function
    Robot robot;   //initialize robot structure and balance
    init_robot(&robot);
    
    bool exploration_complete = false;     //initialize control flags
    bool reached_target = false;
    bool should_exit = false;
    
    while (wb_robot_step(TIME_STEP) != -1 && !should_exit) {   //main control loop
        read_sensors(&robot);  //update sensor readings 
        update_position(&robot); //update position
        
        if (!exploration_complete) {   
            update_exploration_status(&robot);  //exploration phase
            check_light_intensity(&robot);
            
            // Check for maze completion
            if (check_maze_completion(&robot)) {
                exploration_complete = true;
                printf("Starting return to Highest light intensity area...\n");
                continue;
            }
            
            // Regular exploration behavior
            if (check_junction(&robot)) {
                examine_junction(&robot);
            }
            
            double left_speed, right_speed;
            update_motor_controls(&robot, &left_speed, &right_speed, false);  //apply motor speeds
            wb_motor_set_velocity(robot.left_motor, left_speed);
            wb_motor_set_velocity(robot.right_motor, right_speed);
        } else {
            // Return navigation mode
            if (!reached_target) {
                // Check if highest light area is reached using sensor readings
                if (check_reached_highest_light(&robot)) {
                    // Stop robot
                    wb_motor_set_velocity(robot.left_motor, 0.0);
                    wb_motor_set_velocity(robot.right_motor, 0.0);
                    
                    // Print completion message
                    printf("Reached Highest light intensity area (%.3f)!\n",
                           robot.high_light_point.intensity);
                           
                    // turn on the LEDS and speaker
                    play_success_pattern(&robot);      
                    
                    reached_target = true;  //set completion flags
                    should_exit = true;
                    continue;
                }
                
                // Continue navigation if target not reached
                double left_speed, right_speed;
                update_motor_controls(&robot, &left_speed, &right_speed, true);
                wb_motor_set_velocity(robot.left_motor, left_speed);
                wb_motor_set_velocity(robot.right_motor, right_speed);
            }
        }
    }
    
    // Ensure robot is stopped before cleanup
    wb_motor_set_velocity(robot.left_motor, 0.0);
    wb_motor_set_velocity(robot.right_motor, 0.0);
    
    wb_robot_cleanup();
    return 0;
}