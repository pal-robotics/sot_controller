// This test runs with reemc robot
// Please remember to load the robot_description on the ros parameters server

# include <sot_controller/sot_controller.h>

using namespace sot_controller;

int main(int argc,char *argv[]){
    /*
        // Seated configuration

        // Arm left
        jointPositions(20) = -0.49552838669100613;
        jointPositions(21) = 0.024631160165058508;
        jointPositions(22) = 0.3587618799256007;
        jointPositions(23) = 1.7700407547314527;
        jointPositions(24) = -0.3221388264942749;
        jointPositions(25) = -0.12734436348362826;
        jointPositions(26) = 0.00845979656312204;
        // Arm right
        jointPositions(27) = -0.1320161376919419;
        jointPositions(28) = 0.3812710840786022;
        jointPositions(29) = 0.10791338229372394;
        jointPositions(30) = 1.3847557170524056;
        jointPositions(31) = 0.12271955295019997;
        jointPositions(32) = -0.11658357530268998;
        jointPositions(33) = 0.07976770941762998;
        // Head
        jointPositions(34) = -0.0007797819167295179;
        jointPositions(35) = -6.391655055159984e-05;
        // Leg left
        jointPositions(6) = 0.00019271088363695223;
        jointPositions(7) = -0.012281374195991613;
        jointPositions(8)= -1.5539225381281545;
        jointPositions(9) = 2.0989627142381795;
        jointPositions(10) = -0.4325825821837508;
        jointPositions(11) = -0.0076794318323346895;
        // Leg right
        jointPositions(12) = -0.0994575641321298;
        jointPositions(13) = 0.038330463911328064;
        jointPositions(14) = -1.572524176478941;
        jointPositions(15) = 2.100818017809862;
        jointPositions(16) = -0.42492891095648416;
        jointPositions(17) = -0.07516505860639641;
        // Torso
        jointPositions(18) = -0.0440104925677645;
        jointPositions(19) = 0.05530074092254829;
    */

    //Init ROS
    ros::init(argc, argv, "sot_controller_test");
    ros::start();

    //Create the initial position
    stdVector_t jointPositions, jointVelocities;
    int stateSize = 18; // It contains also the ff pose
    jointPositions.resize(stateSize);
    jointVelocities.resize(stateSize);
    for(unsigned int i=0; i<stateSize;i++){
        jointPositions[i] = 0.0;
        jointVelocities[i] = 0.0;
    }

    // Create the controller and the dynamic graph
    SotController controller;
    ros::NodeHandle controller_nh("sot_controller");
    controller.startupPythonEnv(controller_nh);

    stdVector_t ffpose = controller.loadFreeFlyer(controller_nh);

    jointPositions[0] = ffpose[0];
    jointPositions[1] = ffpose[1];
    jointPositions[2] = ffpose[2];
    jointPositions[3] = ffpose[3];
    jointPositions[4] = ffpose[4];
    jointPositions[5] = ffpose[5];

    // Take the device
    SotDevice* device = controller.getDevicePtr();


    // Start the device
    device->init(jointPositions.size());
    device->starting(jointPositions,jointVelocities);
    device->startThread();

    std::vector<double> position(24);
    std::vector<double> velocity(24);

    // Trigger the device computation.
    ros::Duration period(0.01);
    while(ros::ok())
    {
        device->runDevice(period);
        device->getSharedState(position, velocity);
//        std::cout << "position 1: " << position[1] << std::endl;

        period.sleep();

    }
    std::cout << "stopping thread" << std::endl;
    device->stopThread();
    std::cout << "spinning" << std::endl;
    ros::spin();
    std::cout << "spinning done" << std::endl;
    return 0;
}
