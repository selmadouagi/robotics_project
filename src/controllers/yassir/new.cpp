// File:          bug_02.cpp
// Author:        Elizabeth Faulkner
// @date:         2026-03

#include "MyRobot.h"

/////////////////////////////////////////////

int main(int argc, char **argv) {

    MyRobot *robot = new MyRobot();

    robot->run();

    delete robot;

    return 0;
}

//////////////////////////////////////////////