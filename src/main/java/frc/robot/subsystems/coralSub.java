// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Configs.CoralWristConfigurations;
import frc.robot.Configs.ElevatorMotorConfigurations;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.Constants.elevatorSetpoints;
import frc.robot.Constants.wristSetpoints;

public class coralSub extends SubsystemBase {
  /** Creates a new elevatorSub. */
  SparkMax rightElevatorMotor,leftElevatorMotor;
  SparkFlex coralWristMotor;
  TalonFX coralRollerMotor;
  SparkClosedLoopController elevatorClosedLoopController, wristClosedLoopController;
  AbsoluteEncoder elevatorAbsEncoder,wristAbsEncoder;
  RelativeEncoder elevatorRelativeEncoder, wristRelativeEncoder;
  ElevatorFeedforward elevatorFeedforward;
  private boolean wasResetByButton = false;


  public coralSub() {


    coralRollerMotor = new TalonFX(15);

    rightElevatorMotor = new SparkMax(Constants.rightElevator, MotorType.kBrushless);
    leftElevatorMotor = new SparkMax(Constants.leftElevator, MotorType.kBrushless);
    coralWristMotor = new SparkFlex(Constants.coralWrist, MotorType.kBrushless);


    coralRollerMotor.setInverted(true);
    coralRollerMotor.setNeutralMode(NeutralModeValue.Brake);

    //Check which elevatorMotor is getting the absolute encoder
    //elevatorAbsEncoder = leftElevatorMotor.getAbsoluteEncoder(); 
    elevatorRelativeEncoder = leftElevatorMotor.getEncoder();
    wristRelativeEncoder = coralWristMotor.getEncoder();
    wristAbsEncoder = coralWristMotor.getAbsoluteEncoder();
    
    //Check if this is gotten from the right elevator controller
    elevatorClosedLoopController = leftElevatorMotor.getClosedLoopController();
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
  public void resetWristEncoder(){
    wristRelativeEncoder.setPosition(0);
  }
  public void moveToSetpointCoralStation(){
    elevatorClosedLoopController.setReference(
      elevatorSetpoints.CoralStation, ControlType.kPosition);
  } 

  public void moveToSetpointWristCoralStation(){
    wristClosedLoopController.setReference(
      wristSetpoints.WristCoralStation, ControlType.kPosition);
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
      elevatorSetpoints.lvl2, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.5);
  } 

  public void moveToSetpointLevelThree(){
    // elevatorClosedLoopController.setReference(
    //   elevatorSetpoints.lvl3, ControlType.kPosition);
    
    elevatorClosedLoopController.setReference(
      elevatorSetpoints.lvl3, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.05);
  } 

  //I want to see if we can use this to see what voltage we can set the elevator to for feed forward
  //If not, we can just see if we do it through testing what is above with the arbFeedForward value
  public void voltageElevator(){
    leftElevatorMotor.setVoltage(0);
  }

  

  
  public void moveToSetpointWristLevelTwoAndThree(){
    wristClosedLoopController.setReference(
      wristSetpoints.lvl2and3, ControlType.kPosition);
  } 

  public void elevatorSetPower(double power){
    leftElevatorMotor.set(power);
  }

  public void elevatorStop(){
    leftElevatorMotor.set(0);
  }

  public double getElevatorAbsEncoder(){
    return elevatorAbsEncoder.getPosition() * 100;
  }

  public void resetEncoder(){
    elevatorRelativeEncoder.setPosition(0);
  }

  public double getElevatorRelEncoder(){
    return elevatorRelativeEncoder.getPosition();
  }

  public double getWristRelEncoder(){
    return wristRelativeEncoder.getPosition();
  }

  public void intakeSetPower(double power){
    coralRollerMotor.set(power);
  }

  public void intakeStop(){
    coralRollerMotor.set(0);
  }

  public void wristSetPower(double power){
    coralWristMotor.set(power);
  }

  public void wristStop(){
    coralWristMotor.set(0);
  }
  

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Elevator Encoder Value:", getElevatorRelEncoder());

    
    
  }
}
