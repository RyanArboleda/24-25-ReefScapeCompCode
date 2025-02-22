// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeSub extends SubsystemBase {
  /** Creates a new AlgaeSub. */
  private SparkMax armMotor, intakeMotor;
  
  public AlgaeSub() {
  armMotor = new SparkMax(Constants.algaeArm, MotorType.kBrushless);
  intakeMotor = new SparkMax(Constants.algaeRollers, MotorType.kBrushless);

  }

  public void moveArm(double power){
    armMotor.set(power);
  }

  public void stopArm(){
    armMotor.set(0);
  }

  public void moveIntake(double power){
    intakeMotor.set(power);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
