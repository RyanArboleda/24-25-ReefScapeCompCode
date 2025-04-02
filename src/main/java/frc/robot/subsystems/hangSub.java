// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Configs.HangMotorConfigurations;

public class hangSub extends SubsystemBase {
  /** Creates a new hangSub. */
  SparkMax leftHangMotor,rightHangMotor;

  public hangSub() {

  leftHangMotor = new SparkMax(Constants.leftHang, MotorType.kBrushless);
  rightHangMotor = new SparkMax(Constants.rightHang, MotorType.kBrushless);
  

  leftHangMotor.configure(
  HangMotorConfigurations.leftHangConfig, 
  ResetMode.kResetSafeParameters, 
  PersistMode.kPersistParameters);

  rightHangMotor.configure(
  HangMotorConfigurations.rightHangConfig, 
  ResetMode.kResetSafeParameters, 
  PersistMode.kPersistParameters);

  }

  public void hangSetPower(double power){
    leftHangMotor.set(power);
  }
  
  public void hangUp(){
    leftHangMotor.set(1);
  }

  public void hangDown(){
    leftHangMotor.set(-1);
  }

  public void hangStop(){
    leftHangMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
