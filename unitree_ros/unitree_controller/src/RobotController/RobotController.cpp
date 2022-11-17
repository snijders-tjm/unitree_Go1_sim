#include "RobotController.h"
#include "StateCommand.h"
#include <tf/transform_datatypes.h>

#define ROBOT_HEIGHT 0.25
#define X_SHIFT_FRONT 0.0
#define X_SHIFT_BACK -0.0

///////////////////////////////////////////////////////////////////////////////
RobotController::RobotController(const float body[], const float legs[])
: state(ROBOT_HEIGHT), command(ROBOT_HEIGHT),
  delta_x(body[0] * 0.5), delta_y(body[1]*0.5 + legs[1]),
  x_shift_front(X_SHIFT_FRONT), x_shift_back(X_SHIFT_BACK),
  restController(default_stance()),
  trotGaitController(default_stance(),0.18, 0.18, 0.02),
  crawlGaitController(default_stance(), 0.55, 0.45, 0.02),
  standController(default_stance())
{
    // body dimensions
    this->body[0] = body[0];
    this->body[1] = body[1];

    // leg dimensions
    this->legs[0] = legs[0];

    this->legs[1] = legs[1];
    this->legs[2] = legs[2];
    this->legs[3] = legs[3];

    state.foot_locations = default_stance();
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> RobotController::default_stance()
{
    // FR, FL, RR, RL
    Eigen::Matrix<float, 3, 4> default_coordinates;
    default_coordinates <<   delta_x + x_shift_front, // FR - x
                             delta_x + x_shift_front, // FL - x
                            -delta_x + x_shift_back,  // RR - x
                            -delta_x + x_shift_back,  // RL - x

                            -delta_y,   // FR - y
                             delta_y,   // FL - y
                            -delta_y,   // RR - y
                             delta_y,   // RL - y

                             0,     // FR - z
                             0,     // FL - z
                             0,     // RR - z
                             0;     // RL - z
    return default_coordinates;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> RobotController::run()
{
    if(state.behavior_state == REST)
    {
        return(restController.run(state,command));
    }
    else if(state.behavior_state == TROT)
    {
        return(trotGaitController.run(state, command)); 
    }
    else if(state.behavior_state == CRAWL)
    {
        return(crawlGaitController.run(state, command)); 
    }
    else if(state.behavior_state == STAND)
    {
        return(standController.run(state, command)); 
    }
    else
    {
        exit(EXIT_FAILURE);
    }
}

///////////////////////////////////////////////////////////////////////////////
void RobotController::joystick_command(const sensor_msgs::Joy::ConstPtr& msg)
{

    if(msg->buttons[0] && state.behavior_state!=REST) { // rest - A button
        command.trot_event = false;
        command.crawl_event = false;
        command.stand_event = false;
        command.rest_event = true;

    } else if (msg->buttons[1] && state.behavior_state!=TROT) { // trot - B button
        command.trot_event = true;
        command.crawl_event = false;
        command.stand_event = false;
        command.rest_event = false;


    } else if(msg->buttons[2] && state.behavior_state!=CRAWL) { // crawl -  X button
        command.trot_event = false;
        command.crawl_event = true;
        command.stand_event = false;
        command.rest_event = false;

    } else if(msg->buttons[3] && state.behavior_state!=STAND) { // stand - Y button
        command.trot_event = false;
        command.crawl_event = false;
        command.stand_event = true;
        command.rest_event = false;
    }

    if(state.behavior_state == REST) {
        restController.updateStateCommand(msg, state, command);
    } else if(state.behavior_state == TROT) {
        trotGaitController.updateStateCommand(msg, state, command);
    } else if(state.behavior_state == CRAWL) {
        crawlGaitController.updateStateCommand(msg, state, command); 
    } else if(state.behavior_state == STAND) {
        standController.updateStateCommand(msg, state, command); 
    } else {
        exit(EXIT_FAILURE);
    }
}

void RobotController::rest_command(const std_msgs::Bool::ConstPtr& msg)
{
    bool event = msg->data;
    if(event) {
        command.trot_event = false;
        command.crawl_event = false;
        command.stand_event = false;
        command.rest_event = true;
    }
}

void RobotController::trot_command(const std_msgs::Bool::ConstPtr& msg)
{
    bool event = msg->data;
    if(event) {
        command.trot_event = true;
        command.crawl_event = false;
        command.stand_event = false;
        command.rest_event = false;
    }
}

void RobotController::crawl_command(const std_msgs::Bool::ConstPtr& msg)
{
    bool event = msg->data;
    if(event) {
        command.trot_event = false;
        command.crawl_event = true;
        command.stand_event = false;
        command.rest_event = false;
    }
}

void RobotController::stand_command(const std_msgs::Bool::ConstPtr& msg)
{
    bool event = msg->data;
    if(event) {
        command.trot_event = false;
        command.crawl_event = false;
        command.stand_event = true;
        command.rest_event = false;
    }
    std::cout << "NU IN ROBOTCONTROLLER stand_command met bool: " << event <<" \n";
//    if(state.behavior_state == REST) {
//        restController.updateStateCommand(msg, state, command);
//    } else if(state.behavior_state == TROT) {
//        trotGaitController.updateStateCommand(msg, state, command);
//    } else if(state.behavior_state == CRAWL) {
//        crawlGaitController.updateStateCommand(msg, state, command);
//    } else if(state.behavior_state == STAND) {
//        standController.updateStateCommand(msg, state, command);
//    } else {
//        exit(EXIT_FAILURE);
//    }
}



///////////////////////////////////////////////////////////////////////////////
void RobotController::change_controller(){
        const char* note;
    if(command.trot_event){
        if (state.behavior_state == TROT){
            ROS_INFO("Already in Trot state!");
        } else{
            if(state.behavior_state == REST){
                state.behavior_state = TROT;
                state.ticks = 0;
                trotGaitController.reset_pid_controller();
                note = messagePrint();
                printf("%s", note);
            }
        }
        command.trot_event = false;
    } else if(command.crawl_event){
        if (state.behavior_state == CRAWL){
            ROS_INFO("Already in Crawl state!");
        } else if(state.behavior_state == REST){
            state.behavior_state = CRAWL;
            crawlGaitController.first_cycle = true;
            state.ticks = 0;
            note = messagePrint();
            printf("%s", note);
        }
        command.crawl_event = false;
    } else if(command.stand_event){
        if (state.behavior_state == STAND){
            ROS_INFO("Already in Stand state!");
        } else if(state.behavior_state == REST){
            state.behavior_state = STAND;
            note = messagePrint();
            printf("%s", note);
            //state.body_local_position[2] = 0.15*0.25;
        }
        command.stand_event = false;
    } else if(command.rest_event){
        if (state.behavior_state == REST){
            ROS_INFO("Already in Rest state!");
        } else{
            state.behavior_state = REST;
            restController.reset_pid_controller();
            note = messagePrint();
            printf("%s", note);
        }
        command.rest_event = false;
    }
}

///////////////////////////////////////////////////////////////////////////////
void RobotController::imu_orientation(const sensor_msgs::Imu::ConstPtr& msg){
    float x = msg->orientation.x;
    float y = msg->orientation.y;
    float z = msg->orientation.z;
    float w = msg->orientation.w;

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    state.imu_roll = roll;
    state.imu_pitch = pitch;
}

///////////////////////////////////////////////////////////////////////////////
void RobotController::cmd_vel_command(const geometry_msgs::Twist::ConstPtr& msg){

//    if(msg){
//        command.trot_event = true;
//        command.crawl_event = false;
//        command.stand_event = false;
//        command.rest_event = false;
//    }
    
    if(state.behavior_state == REST) {
        restController.updateVelCommand(msg, state, command);
    } else if(state.behavior_state == TROT) {
        trotGaitController.updateVelCommand(msg, state, command);
    } else if(state.behavior_state == CRAWL) {
        crawlGaitController.updateVelCommand(msg, state, command); 
    } else if(state.behavior_state == STAND) {
        standController.updateVelCommand(msg, state, command); 
    } else {
        exit(EXIT_FAILURE);
    }
    
}

const char* RobotController::messagePrint(){
    const char* note;
    if (command.rest_event){
        note = R"(

NOW IN RESTING MODE

L joystick UP/DOWN: change body_local_position x-axis
L joystick L/R:     change body_local_position y-axis
dpad UP/DOWN:       change body_local_position z-axis

dpad L/R:           change body local orientation x-axis
R joystick UP/DOWN: change body local orientation y-axis
R joystick L/R:     change body local orientation z-axis

start button:       change USE_IMU TRUE/FALSE

Rest =  A Button
Trot =  B Button
Crawl = X button
Stand = Y button

CTRL-C / XBOX button to quit
        )";
    } else if (command.trot_event){
        note = R"(

NOW IN TROTTING MODE

L joystick UP/DOWN: change x-axis velocity
L joystick L/R:     change y-axis velocity
R joystick L/R:     change z-axis orientation

start button:       change USE_IMU TRUE/FALSE
back button:        change AUTO_REST TRUE/FALSE

Rest =  A Button
Trot =  B Button
Crawl = X button
Stand = Y button

CTRL-C / XBOX button to quit
)";
    } else if (command.stand_event){
        note = R"(

NOW IN STANDING MODE

L joystick UP/DOWN: change FL foot x-pos
L joystick L/R:     change FL foot y-pos

R joystick UP/DOWN: change FR foot x-pos
R joystick L/R:     change FR foot y-pos

dpad L/R:           change F foot z-pos

Rest =  A Button
Trot =  B Button
Crawl = X button
Stand = Y button

CTRL-C / XBOX button to quit
)";
    } else{
        note = R"(

NOW IN CRAWLING MODE (TODO)

L joystick UP/DOWN: change FL foot x-position
L joystick L/R:     change FL foot y-position

R joystick UP/DOWN: change FR foot x-position
R joystick L/R:     change FR foot y-position

Rest =  A Button
Trot =  B Button
Crawl = X button
Stand = Y button

CTRL-C / XBOX button to quit
)";
    }
    return note;
}
