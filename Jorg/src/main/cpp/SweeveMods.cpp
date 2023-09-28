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
    ConfigureModules(Part::drive);
    ConfigureModules(Part::turn);
}

void SweeveMods::ConfigureModules(const Part &type, double Offset)
{
    switch (type)
    {
    case Part::drive:
        mSwerveBits.mDriveMotor.SetInverted(ChassisConstants::DriveMotorDirection);
        break;

    case Part::turn:
        mSwerveBits.mTurnMotor.SetInverted(ChassisConstants::TurnMotorDirection);
        mturnppid.EnableContinuousInput(units::radian_t(-std::numbers::pi), units::radian_t(std::numbers::pi)); // not sure about the logic here
        break;

    case Part::coder:
        mSwerveBits.mCanCoder.ConfigMagnetOffset(Offset);
        break;
    }
}

void SweeveMods::SetSwerveModuleState(const frc::SwerveModuleState &refstate)
{
    const auto state = frc::SwerveModuleState::Optimize(refstate, units::degree_t(mSwerveBits.mCanCoder.GetAbsolutePosition()));
    const auto angleIWANT{state.angle.Radians()};
    const auto speedIWANT{state.speed};
    const double turnoutput = mturnppid.Calculate(units::radian_t{mSwerveBits.mCanCoder.GetAbsolutePosition() * 1 / 180, angleIWANT});
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
            units::radian_t(GetAngle() * 1/180)};
}
// void SweeveMods::O
double SweeveMods::GetAngle()
{
    return mSwerveBits.mCanCoder.GetAbsolutePosition();
}

double SweeveMods::GetSpeedmps()
{
    return (mSwerveBits.mDriveMotor.GetSelectedSensorVelocity() * GearRatios::DriveBox) / 60;
}