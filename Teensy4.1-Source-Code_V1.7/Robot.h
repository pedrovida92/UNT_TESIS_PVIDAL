#ifndef ROBOT_H
#define ROBOT_H

#ifndef PLANNING_H_
#define TRAJ_NONE_FLAG		0x0
#define TRAJ_PTP_FLAG		0x1
#define TRAJ_LIN_FLAG		0x2
#define TRAJ_CIRC_FLAG		0x3
#define TRAJ_FINISH_FLAG	0x10
#endif

class Robot {
    private:
        bool state;
    public:
        Robot();
        ~Robot();
};

Robot::Robot() : state(TRAJ_NONE_FLAG) {}

Robot::~Robot() {}









#endif  /* ROBOT_H */