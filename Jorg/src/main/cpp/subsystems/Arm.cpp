// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

Arm::Arm(ArmList& armList)
    // The PIDController used by the subsystem
    : PIDSubsystem{frc2::PIDController{0, 0, 0}} 
    , mArmList(armList)
    {}

void Arm::UseOutput(double , double ) {
  // Use the output here
}

double Arm::GetMeasurement() {
  // Return the process variable measurement here
  return 0;
}
