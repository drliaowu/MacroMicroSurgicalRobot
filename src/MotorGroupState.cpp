#include "../include/MotorGroupState.hpp"

MotorGroupState::MotorGroupState()
{
    this->ProximalPanAngle = 0;
    this->ProximalTiltAngle = 0;
    this->DistalPanAngle = 0;
    this->DistalTiltAngle = 0;
}

MotorGroupState::MotorGroupState(int proximalPanAngle, int proximalTiltAngle, int distalPanAngle, int distalTiltAngle)
{
    this->ProximalPanAngle = proximalPanAngle;
    this->ProximalTiltAngle = proximalTiltAngle;
    this->DistalPanAngle = distalPanAngle;
    this->DistalTiltAngle = distalTiltAngle;
}

MotorGroupState MotorGroupState::operator+(const MotorGroupState& other)
{
    MotorGroupState state(this->ProximalPanAngle, this->ProximalTiltAngle, this->DistalPanAngle, this->DistalTiltAngle);

    state.ProximalPanAngle += other.ProximalPanAngle;
    state.ProximalTiltAngle += other.ProximalTiltAngle;
    state.DistalPanAngle += other.DistalPanAngle;
    state.DistalTiltAngle += other.DistalTiltAngle;

    return state;
}

void MotorGroupState::operator+=(const MotorGroupState& other)
{
    this->ProximalPanAngle += other.ProximalPanAngle;
    this->ProximalTiltAngle += other.ProximalTiltAngle;
    this->DistalPanAngle += other.DistalPanAngle;
    this->DistalTiltAngle += other.DistalTiltAngle;
}
