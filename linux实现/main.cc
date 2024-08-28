/**
  ******************************************************************************
  * @file    B03/Path_Planning_Algorithm
  * @author  Zhang weizhi
  * @version V1.0.0
  * @date    14-August-2024
  * @brief   This algorithm is based on the Astar algorithm, and uses the Astar algorithm to do local path planning, 
  * which is suitable for the future space-time position of the aircraft with known obstacles. This algorithm solves: 
  * 1, reading speed from Linux SDcard priority problem, using cyclebuffer management of obstacle plane space-time table. 
  * 2, the direction vector of local planning always points to the latest current target position. 
  * 3, For path planning with sampling time lag, simple prediction is made based on IMU velocity information. 
  * 4, controlled the stable mavlink message sending interval, to ensure a steady stream of position updates. 
  * 5, set the state machine automatic switching, leading segment, obstacle avoidance segment, end segment. 
  * 6, For the position of particle given in the space-time table of obstacle aircraft, particle volume expansion is carried out to simulate the real obstacle. 
  * 7, the equivalent Manhattan body represents the closure range of its tangent circle, speeding up the traversal of the data structure. 
  * 8, The 2D Astar algorithm is reworked into 3D Astar, and the floating point number of actual GPS coordinates is reshaped according to the quantization accuracy, 
  *    which accelerates the 3D Astar wayfinding efficiency. 
  * 9, Due to the unidirectional flow of time, the ingenious idea of dynamic pop-up history obstacle plane vector scale is adopted to ensure a low memory overhead, 
  *    and dynamic switches are adopted when reading file descriptors. Theoretically, there is no upper limit for obstacle plane. 
  * 10, Solve the problem that there are obstacles adjacent to the starting and ending points in local planning leading to no solution, 
  *     and optimize the mechanism to ensure that there is always a solution in time and space.
  ******************************************************************************
*/

#include "algorithmmng.h"


int main(int argc, char const *argv[]){
    
    AlgorithmMng am;
    printf("Fuck onemore run!!!!!!!!!!!!!!!!!!!!!!\n");
    am.start();

    am.stop();
    printf("SUCCESS finished all jobs\n");
    return 0;
}

