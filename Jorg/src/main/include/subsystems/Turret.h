// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/PIDSubsystem.h>
#include <ctre/Phoenix.h>

class Turret : public frc2::PIDSubsystem {
 public:

 struct TurretSystems
 {
  WPI_TalonFX mTurretMotor;
  WPI_PigeonIMU mTurretGyro;
 };

  Turret(TurretSystems& turretSystems);

 protected:
  void UseOutput(double output, double setpoint) override;

  double GetMeasurement() override;

  TurretSystems& mTurretSystems;
};
