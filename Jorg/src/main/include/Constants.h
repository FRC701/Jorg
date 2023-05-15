// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
#include <units/length.h>
#include <units/time.h>
#include <numbers>
#include <ctre/Phoenix.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

namespace OperatorConstants
{

    constexpr int kDriverControllerPort = 0;

} // namespace OperatorConstants

namespace ChassisConstants
{

    static constexpr int kFrontLeft = 1;
    static constexpr int kFrontRight = 2;
    static constexpr int kRearLeft = 3;
    static constexpr int kRearRight = 4;

    static constexpr int kFrontLeftMod = 1;
    static constexpr int kFrontRightMod = 2;
    static constexpr int kRearLeftMod = 3;
    static constexpr int kRearRightMod = 4;

    static constexpr int kFrontLeftTurnMod = 5;
    static constexpr int kFrontRightTurnMod = 6;
    static constexpr int kRearLeftTurnMod = 7;
    static constexpr int kRearRightTurnMod = 8;

    static constexpr int kFrontLeftCoder = 9;
    static constexpr int kFrontRightCoder = 10;
    static constexpr int kRearLeftCoder = 11;
    static constexpr int kRearRightCoder = 12;

    const auto DriveMotorDirection = ctre::phoenix::motorcontrol::TalonFXInvertType::Clockwise;
    const auto TurnMotorDirection = ctre::phoenix::motorcontrol::TalonFXInvertType::Clockwise;
    const double CanCoderOffset = 0;
    static const auto MaxAngularVelocity = std::numbers::pi * 1_rad_per_s;
    static const auto ModuleMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s / 1_s;
    const double TOPSPEED = 4.877; //mps
}

namespace GearRatios
{
    const double FalconEncodertoDegs {4096/360};
    const double TractionWheelDiameter{0.127};
    const double TractionWheelCirumference{2 * std::numbers::pi * TractionWheelDiameter};
    const double DriveBox{6.75 * TractionWheelCirumference};
}