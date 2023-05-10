// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SweeveMods.h"
#include <Constants.h>


SweeveMods::SweeveMods(SwerveBits &swerveBits)
    : mdrivepid{0.1, 0, 0}
    , mturnppid{1.0, 0.0, 0.0, 
                            {ChassisConstants::MaxAngularVelocity, ChassisConstants::ModuleMaxAngularAcceleration}},
     FFcontrol{1_V, 0.5_V / 1_rad_per_s}
     , mSwerveBits(swerveBits)
{
        mSwerveBits.mDriveMotor.SetInverted(ChassisConstants::DriveMotorDirection);
         mSwerveBits.mTurnMotor.SetInverted(ChassisConstants::TurnMotorDirection);
        mturnppid.EnableContinuousInput(units::radian_t(-std::numbers::pi), units::radian_t(std::numbers::pi)); 
        mSwerveBits.mCanCoder.ConfigMagnetOffset(ChassisConstants::CanCoderOffset);
}

void SweeveMods::SetSwerveModuleState(const frc::SwerveModuleState &refstate)
{
    const auto state = frc::SwerveModuleState::Optimize(refstate, units::radian_t(GetAngle()));
    const auto angleIWANT{state.angle.Radians()};
    const auto speedIWANT{state.speed};
    const double turnoutput = mturnppid.Calculate(units::radian_t{GetAngle().value()}, angleIWANT);
    const auto feedforwardTurnOutput = FFcontrol.Calculate(mturnppid.GetSetpoint().velocity);
    const double driveoutput = mdrivepid.Calculate(GetSpeedmps(), speedIWANT.value());

    mSwerveBits.mDriveMotor.SetVoltage(units::volt_t(driveoutput));
    mSwerveBits.mTurnMotor.SetVoltage(units::volt_t(turnoutput) + feedforwardTurnOutput);
}


/*frc::SwerveModuleState SweeveMods::GetCurrentState() {
    return {units::meters_per_second_t{GetSpeedmps()}, 
            units::radian_t(GetAngle() * 1/180)};
}*/ //unused function

frc::SwerveModulePosition SweeveMods::GetCurrentPosition() {
    return {units::meter_t{mSwerveBits.mDriveMotor.GetSelectedSensorPosition() * GearRatios::DriveBox}, 
            units::radian_t(GetAngle())};
}
// void SweeveMods::O
units::degree_t SweeveMods::GetAngle()
{
    return units::degree_t{mSwerveBits.mCanCoder.GetAbsolutePosition()};
}

double SweeveMods::GetSpeedmps()
{
    return ((mSwerveBits.mDriveMotor.GetSelectedSensorVelocity() * 10) / 2048) / GearRatios::DriveBox * GearRatios::TractionWheelCirumference;
}