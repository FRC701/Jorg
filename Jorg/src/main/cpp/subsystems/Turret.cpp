// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include "subsystems/Turret.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

Turret::Turret(TurretSystems& turretSystems)
    // The PIDController used by the subsystem
    : PIDSubsystem{frc2::PIDController{0, 0, 0}} 
    , mTurretSystems(turretSystems)
    {
      m_controller.EnableContinuousInput(0, 360);
    }

void Turret::UseOutput(double output, double setpoint) 
{
  double PIDOUTPUT = m_controller.Calculate(GetMeasurement(), setpoint);
  mTurretSystems.mTurretMotor.Set(PIDOUTPUT);
}

double Turret::GetMeasurement() 
{ 
  return remainder(mTurretSystems.mTurretMotor.GetSelectedSensorPosition(), GearRatios::TurretGearRatio * 2048) / 
  (GearRatios::TurretGearRatio * 2048 /360);
}

double Turret::PowerSet(double speed)
{
  mTurretSystems.mTurretMotor.Set(speed);
  return speed;
}

void Turret::Periodic()
{
  frc::SmartDashboard::PutNumber("Turret Angle", GetMeasurement());
}