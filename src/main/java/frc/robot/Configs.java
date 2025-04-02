// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.LimelightHelpers.RawFiducial;

/** Add your docs here. */
public class Configs {

    public static final class ElevatorMotorConfigurations{
        public static final SparkMaxConfig leftElevatorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightElevatorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig coralWristConfig = new SparkMaxConfig();

        static{
            //idk smart lim current or voltage compensation so like change that ig if we need it
            //basic settings
            leftElevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12);
            rightElevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12);

            //limit switch
            leftElevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true).reverseLimitSwitchType(Type.kNormallyOpen);
            rightElevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true).reverseLimitSwitchType(Type.kNormallyOpen);

            
            rightElevatorConfig.follow(Constants.leftElevator, true);
            leftElevatorConfig.inverted(false);
            
            
            //same thing as a long line of code btw
            //closed loop controller
            leftElevatorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .i(0)
            .d(6)
            .velocityFF(0.5)
            .outputRange(-0.5, 0.5)
            .maxMotion
            .maxVelocity(2100)
            .maxAcceleration(1500)
            .allowedClosedLoopError(0.1);

            rightElevatorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .i(0)
            .d(6)
            .velocityFF(0.5)
            .outputRange(-0.5, 0.5)
            .maxMotion
            .maxVelocity(2100)
            .maxAcceleration(1500)
            .allowedClosedLoopError(0.1);

        }


    }

    public static final class CoralWristConfigurations{
        public static final SparkMaxConfig coralWristConfig = new SparkMaxConfig();

        static {

            coralWristConfig.limitSwitch.reverseLimitSwitchEnabled(true).reverseLimitSwitchType(Type.kNormallyOpen);


            coralWristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

            coralWristConfig.
            closedLoop.
            feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.5)
            .outputRange(-0.5, 0.5)
            .maxMotion
            .maxVelocity(4200)
            .maxAcceleration(6000)
            .allowedClosedLoopError(0.25);


        }
        
    }

    public static final class HangMotorConfigurations{
        public static final SparkMaxConfig leftHangConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightHangConfig = new SparkMaxConfig();
static{
        leftHangConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12);
        rightHangConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12);


        rightHangConfig.follow(Constants.leftHang, true);
        leftHangConfig.inverted(true);
        }
    }

    public static final class AlgaeMotorConfigurations{
        public static final SparkMaxConfig algaeArmConfig = new SparkMaxConfig();
        public static final SparkMaxConfig algaeRollersConfig = new SparkMaxConfig();

static{

    algaeRollersConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

    algaeArmConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
    algaeArmConfig.inverted(true);

    algaeArmConfig.
    closedLoop.
    feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1)
    .outputRange(-0.5, 0.5)
    .maxMotion
    .maxVelocity(2000)
    .maxAcceleration(10000)
    .allowedClosedLoopError(0.25);
}

    }

}
