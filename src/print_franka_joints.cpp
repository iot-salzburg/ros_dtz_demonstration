#include <cmath>
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <stdio.h>
#include <unistd.h>

// Prints the joint values of the current franka panda robot position

int main(int argc, char** argv) {
    bool print = false;
    franka::Robot robot(argv[1]);
  try {
        size_t count = 0;
        std::array<double, 7> currentPosition;



        robot.read([&print, &count, &currentPosition](const franka::RobotState& robot_state) {

            if (!print){
                print = true;
                std::cout << "Measured joint positions:" << std::endl;
                std::cout << "{" << std::endl;
                for (int i = 0; i < 7; i++){
                    if (i < 6){
                         printf("%+f,\t\t// Joint %i", robot_state.q[i],i+1);
                    } else {
                         printf("%+f\t\t// Joint %i", robot_state.q[i],i+1);
                    }
                    std::cout << std::endl;
                }
                std::cout << "};" << std::endl;
                currentPosition = robot_state.q;
                return count++ < 1;
            }
            return false;;

        });

        /*
        std::array<double, 7> piFactors;
        
        for(int i=0; i<7; i++){
            piFactors[i] = currentPosition[i] * 180 / M_PI;
        }

        std::cout << "Calculated Pi Factors" << std::endl;
        for (int i = 0; i < 7; i++){
            printf("[%i] %+f \n", i, piFactors[i]);
        }

        for(int i=0; i<7; i++){
            piFactors[i] = round(piFactors[i]);
        }

        std::cout << "Rounded Pi Factors" << std::endl;
        for (int i = 0; i < 7; i++){
            printf("[%i] %+f \n", i, piFactors[i]);
        }
    */
    } catch (const franka::Exception& ex) {
        std::cerr << ex.what() << std::endl;
        std::cin.ignore();
        robot.automaticErrorRecovery();
  }
  

  std::cout << "Press Any Key to close.." << std::endl;
  std::cin.ignore();
  return 0;
    
}