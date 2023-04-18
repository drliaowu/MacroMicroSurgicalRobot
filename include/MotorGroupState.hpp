#ifndef MOTOR_GROUP_STATE
#define MOTOR_GROUP_STATE

class MotorGroupState
{
public:
    int ProximalPanAngle;
    int ProximalTiltAngle;
    int DistalPanAngle;
    int DistalTiltAngle;

    MotorGroupState();
    MotorGroupState(int proximalPanAngle, int proximalTiltAngle, int distalPanAngle, int distalTiltAngle);
    MotorGroupState operator+(const MotorGroupState& other);
    void operator+=(const MotorGroupState& other);
};
#endif
