
#ifndef PD_CONTROLLER_HPP
#define PD_CONTROLLER_HPP


struct PDController
{

    double kp_, kd_, dt_;

    PDController(double kp, double kd, double dt) : kp_(kp), kd_(kd), dt_(dt) {}

    double get_output(double r, double y, double dy) {
        return kp_ * (r - y) + kd_ * dy;
    }

};


#endif
