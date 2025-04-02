// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    //Controllers
    public static int driver1 = 0;
    public static int codriver = 1;

    //Motor IDs
    public static int leftElevator = 7;
    public static int rightElevator = 2;
    public static int leftHang = 4;
    public static int rightHang = 5;

    //Don't have this one anymore
    public static int coralWrist = 6; 

    public static int coralRollers = 15; //This one is changed because it is in the Phoenix Tuner
    public static int algaeArm = 3;
    public static int algaeRollers = 1;


    public static double percentSlow = 0.35;
    public static double dPadSlow = 0.95;

    //visoin
    public static class VisionConstants{
    
    public static final String LimeLightName = "";

    public static double reefAlignX = 0.15;
    public static double reefAlignY = -0.5;
    public static double reefAlignRot = 0.03;

    public static double reefAlignRotSetpos = 0;
    public static double reefAlignRotTolerance = 0.5;
    
    public static double reefAlignXSetpos = -0.5;
    public static double reefAlignTXolerance = 0.005;

    public static double reefAlignYSetpos = 0.4;
    public static double reefAlignYTolerance = 0.1;
    
    public static final double waitTime = 1;
    public static final double validationTime = 0.3;

    public static final double reefAngle = 22d; //degrees ig
    public static final double reefTolerance = 2.2;

    public static final double tolerance = 0.01;
    }

    public static class elevatorSetpoints{
        public static double CoralStation = .1;
        public static double lvl1 = 0.05;
        public static double lvl2 = 5.58;
        public static double lvl3 = 21;
    }

    public static class wristSetpoints{
        public static double WristCoralStation = 2.46;
        public static double lvl1 = 1;
        public static double lvl2and3 = -0.41;
    }

    public static class algaeSetpoints{
        public static double algaeArmOutPos = -2.9;
        public static double algaeArmInPos = 0;
        public static double algaeArmCheckPos = -1.29;
    }

    public static class SimulationRobotConstants{
        public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass =
        4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.922; // m
    public static final double kMaxElevatorHeightMeters = 1.62; // m

    public static final double kArmReduction = 60; // 60:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads =
        Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads =
        Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = 0.4032262; // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
    public static final double kIntakeShortBarLength = 0.1524;
    public static final double kIntakeLongBarLength = 0.3048;
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
    }

}
