// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/PIDSubsystem.h>
#include <ctre/Phoenix.h>
#include <frc/controller/ArmFeedforward.h>

class Arm : public frc2::PIDSubsystem {
 public:

struct ArmList
{
  WPI_TalonFX& mArmMotor;
  CANCoder& mArmCanCoder;
};
Arm(ArmList& ArmList);

 protected:
  void UseOutput(double output, double setpoint) override;

  double GetMeasurement() override;

  ArmList& mArmList;
};
