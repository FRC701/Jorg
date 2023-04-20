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
    const auto DriveMotorDirection = ctre::phoenix::motorcontrol::TalonFXInvertType::Clockwise;
    const auto TurnMotorDirection = ctre::phoenix::motorcontrol::TalonFXInvertType::Clockwise;
    const double CanCoderOffset = 0;

}

namespace GearRatios
{
    const double TractionWheelDiameter{0.127};
    const double TractionWheelCirumference{2 * std::numbers::pi * TractionWheelDiameter};
    const double DriveBox{6.75 * TractionWheelCirumference};
}