// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class hangSub extends SubsystemBase {
  /** Creates a new hangSub. */
  TalonFX leftHangMotor,rightHangMotor;

  public hangSub() {

  leftHangMotor = new TalonFX(Constants.leftHang);
  rightHangMotor = new TalonFX(Constants.rightHang);

  }


  public void hangUp(){
    leftHangMotor.set(100);
    rightHangMotor.set(100);
  }

  public void hangDown(){
    leftHangMotor.set(-100);
    rightHangMotor.set(-100);
  }

  public void hangStop(){
    leftHangMotor.set(0);
    rightHangMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
