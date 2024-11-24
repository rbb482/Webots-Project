#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>
#include <webots/gps.h>
#include <webots/light_sensor.h>
#define MAX_SENSOR_NUMBER 16
#define RANGE (1024 / 2)
#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))
#define MAX_PATH_LENGTH 1000
#define JUNCTION_THRESHOLD 0.7
#define GPS_THRESHOLD 0.05 // Threshold for considering the robot back at the start point
#define LIGHT_THRESHOLD 0.03 // Threshold for considering the robot back at the start point
#define START_DELAY 1000   // Delay before starting movement (in milliseconds)
static double max_light_value = 0.0;
static double max_light_x = 0.0;
static double max_light_y = 0.0;
static double max_light_z = 0.0;
typedef struct {
    double x;
    double y;
} Position;

static WbDeviceTag sensors[MAX_SENSOR_NUMBER], left_motor, right_motor;
static WbDeviceTag left_position_sensor, right_position_sensor, gps;
static double matrix[MAX_SENSOR_NUMBER][2];
static int num_sensors;
static double range;
static int time_step = 0;
static double max_speed = 0.0;
static double speed_unit = 1.0;


static Position start_position;
static int start_time;
static int has_moved = 0;
#define CHECK_START_DELAY 60000
static Position path[MAX_PATH_LENGTH];
static int path_length = 0;
static int junction_count = 0;
static int current_direction = 0; // 0: forward, 1: left, 2: right, 3: backward

static WbDeviceTag light_sensor;
static FILE *data_log;

static void initialize() {
    wb_robot_init();

    time_step = wb_robot_get_basic_time_step();

    const char *robot_name = wb_robot_get_name();

    const char e_puck_name[] = "ps0";


    char sensors_name[5];
    const double(*temp_matrix)[2];


    range = RANGE;
    light_sensor = wb_robot_get_device("light sensor");
wb_light_sensor_enable(light_sensor, time_step);

    const double e_puck_matrix[8][2] = {{150, -35}, {100, -15}, {80, -10},  {-10, -10},
                                        {-10, -10}, {-10, 80},  {-30, 100}, {-20, 150}};
    

    if (strncmp(robot_name, "e-puck", 6) == 0) {
        const double epuck_max_speed = 6.28;
        const double epuck_speed_unit = 0.00628;

        num_sensors = 8;
        sprintf(sensors_name, "%s", e_puck_name);
        temp_matrix = e_puck_matrix;
        max_speed = epuck_max_speed;
        speed_unit = epuck_speed_unit;
    } else {
        fprintf(stderr, "This controller doesn't support this robot\n");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < num_sensors; i++) {
        sensors[i] = wb_robot_get_device(sensors_name);
        wb_distance_sensor_enable(sensors[i], time_step);

        if ((i + 1) >= 10) {
            sensors_name[2] = '1';
            sensors_name[3]++;

            if ((i + 1) == 10) {
                sensors_name[3] = '0';
                sensors_name[4] = '\0';
            }
        } else
            sensors_name[2]++;

        for (int j = 0; j < 2; j++)
            matrix[i][j] = temp_matrix[i][j];
    }

    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);

    left_position_sensor = wb_robot_get_device("left wheel sensor");
    right_position_sensor = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(left_position_sensor, time_step);
    wb_position_sensor_enable(right_position_sensor, time_step);

 

    // Initialize GPS
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, time_step);
    
    data_log = fopen("sensor_gps_log.csv", "w");
if (data_log == NULL) {
    printf("Error opening log file!\n");
    exit(1);
}
fprintf(data_log, "Time,Light,GPS_X,GPS_Y,GPS_Z\n");

    // Record start time
    start_time = wb_robot_get_time();

    printf("The %s robot is initialized, it uses %d distance sensors and GPS\n", robot_name, num_sensors);
}

static void update_path_memory() {
    double left_pos = wb_position_sensor_get_value(left_position_sensor);
    double right_pos = wb_position_sensor_get_value(right_position_sensor);
    
    double x = (left_pos + right_pos) / 2 * cos(current_direction * M_PI / 2);
    double y = (left_pos + right_pos) / 2 * sin(current_direction * M_PI / 2);

    if (path_length < MAX_PATH_LENGTH) {
        path[path_length].x = x;
        path[path_length].y = y;
        path_length++;
    }
}

static int detect_junction(double *sensors_value) {
    int left_open = 0, right_open = 0, front_open = 1;

    for (int i = 0; i < num_sensors; i++) {
        double normalized_value = 1.0 - (sensors_value[i] / range);
        if (normalized_value > JUNCTION_THRESHOLD) {
            if (i < num_sensors / 3) right_open = 1;
            else if (i > 2 * num_sensors / 3) left_open = 1;
            else front_open = 0;
        }
    }

    return (left_open + right_open + front_open) >= 2;
}

static void analyze_junction(double *sensors_value) {
    int left_open = 0, right_open = 0, front_open = 1;

    for (int i = 0; i < num_sensors; i++) {
        double normalized_value = 1.0 - (sensors_value[i] / range);
        if (normalized_value > JUNCTION_THRESHOLD) {
            if (i < num_sensors / 3) right_open = 1;
            else if (i > 2 * num_sensors / 3) left_open = 1;
            else front_open = 0;
        }
    }

    if (left_open && current_direction != 3) {
        current_direction = 1; // Turn left
    } else if (right_open && current_direction != 1) {
        current_direction = 3; // Turn right
    } else if (front_open) {
        current_direction = 0; // Go straight
    } else {
        current_direction = 2; // Turn back
    }

    junction_count++;
}

static int is_back_at_start() {
    const double *gps_values = wb_gps_get_values(gps);
    double dx = gps_values[0] - start_position.x;
    double dy = gps_values[1] - start_position.y;
    double distance = sqrt(dx*dx + dy*dy);
    printf("S: (%.2f, %.2f,%.2f,%.2f)\n",dx,dy,distance,GPS_THRESHOLD);
    return distance < GPS_THRESHOLD;
}

static int max_light_spot() {
    const double *gps_values = wb_gps_get_values(gps);
    double dx = gps_values[0] - max_light_x;
    double dy = gps_values[1] - max_light_y;
    double distance = sqrt(dx*dx + dy*dy);
    printf("S: (%.2f, %.2f,%.2f,%.2f)\n",dx,dy,distance,LIGHT_THRESHOLD);
    return distance < LIGHT_THRESHOLD;
}

static void record_start_position() {
    const double *gps_values = wb_gps_get_values(gps);
    start_position.x = gps_values[0];
    start_position.y = gps_values[1];
    printf("Start position recorded: (%.2f, %.2f)\n", start_position.x, start_position.y);
}

static void log_sensor_and_gps_data() {

    double light_value = wb_light_sensor_get_value(light_sensor);
    const double *gps_values = wb_gps_get_values(gps);
    double current_time = wb_robot_get_time();
    
    // Update max light value if necessary
    if (light_value > max_light_value) {
        max_light_value = light_value;
        max_light_x = gps_values[0];
        max_light_y = gps_values[1];
        max_light_z = gps_values[2];
    }
    
    fprintf(data_log, "%.2f,%.2f,%.6f,%.6f,%.6f\n", 
            current_time, light_value, gps_values[0], gps_values[1], gps_values[2]);
    
    // Print to console
    printf("Time: %.2f, Light: %.2f, GPS: (%.6f, %.6f, %.6f)\n", 
           current_time, light_value, gps_values[0], gps_values[1], gps_values[2]);
}

void delay(int milliseconds) {
    int start_time = wb_robot_get_time() * 1000;  // Get current simulation time in milliseconds
    while (wb_robot_step(wb_robot_get_basic_time_step()) != -1) {
        int current_time = wb_robot_get_time() * 1000;  // Get updated simulation time
        if (current_time - start_time >= milliseconds) {
            break;  // Exit the loop after the specified delay time
        }
    }
}

int main() {
    initialize();
    
    double start_check_time = wb_robot_get_time() * 1000 + CHECK_START_DELAY;
    
    while (wb_robot_step(time_step) != -1) {
        double speed[2] = {0.0, 0.0};
        double sensors_value[MAX_SENSOR_NUMBER];
        
        // Check if we should start moving
        if (wb_robot_get_time() * 1000 < start_time + START_DELAY) {
            if (!has_moved) {
                record_start_position();
                has_moved = 1;
            }
            continue;  // Skip the rest of the loop
        }


        for (int i = 0; i < num_sensors; i++)
            sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);

        update_path_memory();
        log_sensor_and_gps_data();

        if (detect_junction(sensors_value)) {
            analyze_junction(sensors_value);
        }

        // Modified Braitenberg algorithm
        for (int i = 0; i < 2; i++) {
            speed[i] = 0.0;

            for (int j = 0; j < num_sensors; j++) {
                speed[i] += speed_unit * matrix[j][i] * (1.0 - (sensors_value[j] / range));
            }

            // Adjust speed based on current_direction
            switch (current_direction) {
                case 1: // Left turn
                    speed[i] *= (i == 0) ? 0.5 : 1.5;
                    break;
                case 2: // Turn back
                    speed[i] *= (i == 0) ? -1 : 1;
                    break;
                case 3: // Right turn
                    speed[i] *= (i == 0) ? 1.5 : 0.5;
                    break;
                default: // Forward
                    break;
            }

            speed[i] = BOUND(speed[i], -max_speed, max_speed);
        }

        // Set the motor speeds
        wb_motor_set_velocity(left_motor, speed[0]);
        wb_motor_set_velocity(right_motor, speed[1]);

        // Check if back at start after 1 minute
        if (wb_robot_get_time() * 700 >= start_check_time) {
            if (is_back_at_start()) {
                printf("Back at start position after 1 minute. Stopping.\n");
                wb_motor_set_velocity(left_motor, 0.0);
                wb_motor_set_velocity(right_motor, 0.0);
                break;  // Exit the main loop
            } else {
                printf("Not back at start position after 1 minute. Continuing.\n");
                // Reset the timer for the next check
                
                continue;
            }
        }
    }
    
    printf("Maximum light value: %.2f\n", max_light_value);
printf("Coordinates of maximum light: (%.6f, %.6f, %.6f)\n", max_light_x, max_light_y, max_light_z);
    fclose(data_log);
     
     
     delay(10000);
     
     
     
     while (wb_robot_step(time_step) != -1) {
        double speed[2] = {0.0, 0.0};
        double sensors_value[MAX_SENSOR_NUMBER];
        
        // Check if we should start moving
        if (wb_robot_get_time() * 1000 < start_time + START_DELAY) {
            if (!has_moved) {
                record_start_position();
                has_moved = 1;
            }
            continue;  // Skip the rest of the loop
        }

       

        for (int i = 0; i < num_sensors; i++)
            sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);

        update_path_memory();
        log_sensor_and_gps_data();

        if (detect_junction(sensors_value)) {
            analyze_junction(sensors_value);
        }

        // Modified Braitenberg algorithm
        for (int i = 0; i < 2; i++) {
            speed[i] = 0.0;

            for (int j = 0; j < num_sensors; j++) {
                speed[i] += speed_unit * matrix[j][i] * (1.0 - (sensors_value[j] / range));
            }

            // Adjust speed based on current_direction
            switch (current_direction) {
                case 1: // Left turn
                    speed[i] *= (i == 0) ? 0.5 : 1.5;
                    break;
                case 2: // Turn back
                    speed[i] *= (i == 0) ? -1 : 1;
                    break;
                case 3: // Right turn
                    speed[i] *= (i == 0) ? 1.5 : 0.5;
                    break;
                default: // Forward
                    break;
            }

            speed[i] = BOUND(speed[i], -max_speed, max_speed);
        }

        // Set the motor speeds
        wb_motor_set_velocity(left_motor, speed[0]);
        wb_motor_set_velocity(right_motor, speed[1]);

        // Check if back at start after 1 minute
       
            if (max_light_spot()) {
                printf("Brightest Spot in Map Reached\n");
                wb_motor_set_velocity(left_motor, 0.0);
                wb_motor_set_velocity(right_motor, 0.0);
                break;  // Exit the main loop
            } else {
                printf("Not back at MAX LIGHT position after 1 minute. Continuing.\n");
                // Reset the timer for the next check
                
                continue;
            }
        
    }
    wb_robot_cleanup();
    return 0;
}
