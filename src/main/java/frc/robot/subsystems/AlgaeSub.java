// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class AlgaeSub extends SubsystemBase {
  /** Creates a new AlgaeSub. */
  SparkMax armMotor, intakeMotor;
  RelativeEncoder algaeRelEncoder;
  SparkClosedLoopController algaeClosedLoop;
  
  public AlgaeSub() {
  armMotor = new SparkMax(Constants.algaeArm, MotorType.kBrushless);
  intakeMotor = new SparkMax(Constants.algaeRollers, MotorType.kBrushless);
  algaeRelEncoder = armMotor.getEncoder();


  armMotor.configure(Configs.AlgaeMotorConfigurations.algaeArmConfig,
  ResetMode.kResetSafeParameters, 
  PersistMode.kPersistParameters);
  intakeMotor.configure(Configs.AlgaeMotorConfigurations.algaeRollersConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void moveArm(double power){
    armMotor.set(power);
  }

  public void stopArm(){
    armMotor.set(0);
  }

  public void intakeSetPower(double power){
    intakeMotor.set(power);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }

  public double getAlgaeEncoder(){
    return algaeRelEncoder.getPosition();
  }

  public void resetAlgaeEncoder(){
    algaeRelEncoder.setPosition(0);
  }

  public void algaeArmInSetpoint(){
    algaeClosedLoop.setReference(
      Constants.algaeSetpoints.algaeArmInPos, ControlType.kPosition);
  } 

  public void algaeArmOutSetpoint(){
    algaeClosedLoop.setReference(
      Constants.algaeSetpoints.algaeArmOutPos, ControlType.kPosition);
  } 
  
  public void algaeArmElevatorCheck(){
    algaeClosedLoop.setReference(
      Constants.algaeSetpoints.algaeArmCheckPos, ControlType.kPosition);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Encoder: ", getAlgaeEncoder());
  }
}
