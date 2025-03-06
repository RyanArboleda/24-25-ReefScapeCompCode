// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Configs.CoralWristConfigurations;
import frc.robot.Configs.ElevatorMotorConfigurations;
import frc.robot.Constants.elevatorSetpoints;
import frc.robot.Constants.wristSetpoints;

public class coralSub extends SubsystemBase {
  /** Creates a new elevatorSub. */
  SparkMax rightElevatorMotor,leftElevatorMotor,coralWristMotor,coralRollerMotor;
  SparkClosedLoopController elevatorClosedLoopController, wristClosedLoopController;
  AbsoluteEncoder elevatorAbsEncoder,wristAbsEncoder;
  private boolean wasResetByButton = false;


  public coralSub() {



    rightElevatorMotor = new SparkMax(Constants.rightElevator, MotorType.kBrushless);
    leftElevatorMotor = new SparkMax(Constants.leftElevator, MotorType.kBrushless);
    coralWristMotor = new SparkMax(Constants.coralWrist, MotorType.kBrushless);
    coralRollerMotor = new SparkMax(Constants.coralRollers, MotorType.kBrushless);

    //Check which elevatorMotor is getting the absolute encoder
    elevatorAbsEncoder = rightElevatorMotor.getAbsoluteEncoder(); 
    wristAbsEncoder = coralWristMotor.getAbsoluteEncoder();
    
    //Check if this is gotten from the right elevator controller
    elevatorClosedLoopController = rightElevatorMotor.getClosedLoopController();
    wristClosedLoopController = coralWristMotor.getClosedLoopController();
    
    rightElevatorMotor.configure(ElevatorMotorConfigurations.rightElevatorConfig,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    leftElevatorMotor.configure(ElevatorMotorConfigurations.leftElevatorConfig,
    ResetMode.kResetSafeParameters, 
    PersistMode.kPersistParameters);

    coralWristMotor.configure(CoralWristConfigurations.coralWristConfig,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters
    );

    
  }

  public void moveToSetpointCoralStation(){
    elevatorClosedLoopController.setReference(
      elevatorSetpoints.CoralStation, ControlType.kPosition);
  } 

  public void moveToSetpointWristCoralStation(){
    wristClosedLoopController.setReference(
      wristSetpoints.CoralStation, ControlType.kPosition);
  }

  public void moveToSetpointLevelOne(){
    elevatorClosedLoopController.setReference(
      elevatorSetpoints.lvl1, ControlType.kPosition);
  } 

  public void moveToSetpointWristLevelOne(){
    wristClosedLoopController.setReference(
      wristSetpoints.lvl1, ControlType.kPosition);
  } 

  public void moveToSetpointLevelTwo(){
    elevatorClosedLoopController.setReference(
      elevatorSetpoints.lvl2, ControlType.kPosition);
  } 

  public void moveToSetpointLevelThree(){
    elevatorClosedLoopController.setReference(
      elevatorSetpoints.lvl3, ControlType.kPosition);
  } 

  
  public void moveToSetpointWristLevelTwoAndThree(){
    wristClosedLoopController.setReference(
      wristSetpoints.lvl2and3, ControlType.kPosition);
  } 

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
