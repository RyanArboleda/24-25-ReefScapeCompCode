// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlgaeArmCheckPos;
import frc.robot.commands.AlgaeArmOutPos;
import frc.robot.commands.CoralLvl1Command;
import frc.robot.commands.CoralLvl2Command;
import frc.robot.commands.CoralLvl3Command;
import frc.robot.commands.CoralStationCommand;
import frc.robot.commands.CoralStop;
import frc.robot.commands.HangArmIn;
import frc.robot.commands.HangArmOut;
import frc.robot.commands.IntakeAlgaeAutomatic;
import frc.robot.commands.IntakeAlgaeCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.ManualAlgaeArmDown;
import frc.robot.commands.ManualAlgaeArmUp;
import frc.robot.commands.ManualElevatorDown;
import frc.robot.commands.ManualElevatorUp;
import frc.robot.commands.SpitAlgaeCommand;
import frc.robot.commands.SpitCoral;
import frc.robot.commands.SpitCoralAuto;
import frc.robot.commands.hangStick;
import frc.robot.commands.zeroElevatorEncoderCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSub;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.coralSub;
import frc.robot.subsystems.hangSub;
import frc.robot.subsystems.visionSub;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double percentSlow = 1;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(Constants.driver1);
    private final CommandXboxController codriver = new CommandXboxController(Constants.codriver);
    private final SlewRateLimiter filterX = new SlewRateLimiter(2.5);
    private final SlewRateLimiter filterY = new SlewRateLimiter(2.5);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static coralSub mCoralSub = new coralSub();

    public static hangSub mHangSub = new hangSub();

    public static AlgaeSub mAlgaeSub = new AlgaeSub();

    private final visionSub m_Vision = new visionSub();



    /* Path follower */
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    public RobotContainer() {

        for (int port = 5800; port <= 5809; port++) {
          PortForwarder.add(port, "limelight.local", port);
      }

        NamedCommands.registerCommand("Coral LVL 3", new CoralLvl3Command());

        NamedCommands.registerCommand("Spit Coral", new SpitCoralAuto());

        NamedCommands.registerCommand("Coral Intake Stop", new CoralStop());
        
        NamedCommands.registerCommand("Coral Station", new CoralStationCommand());

        NamedCommands.registerCommand("Shoot", getAutonomousCommand());
        

        autoChooser.setDefaultOption("OurTestAuto", new PathPlannerAuto("OurTestAuto"));
        // autoChooser = AutoBuilder.buildAutoChooser("OurTestAuto");
        // Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);
    
        // autoChooser.addOption("OurTestAuto", new PathPlannerAuto("OurTestAuto"));
        autoChooser.addOption("East Mid Coral Auto", new PathPlannerAuto("East Mid Coral Auto"));
        autoChooser.addOption("NorthEast Far Auto", new PathPlannerAuto("NorthEast Far Auto"));
        autoChooser.addOption("SouthEast Proc Auto", new PathPlannerAuto("SouthEast Proc Auto"));
        autoChooser.addOption("SouthWest Mid Auto", new PathPlannerAuto("SouthWest Mid Auto"));

        // autoChooser = AutoBuilder.buildAutoChooser("Tests");

        // autoChooser.addOption("OurTestAuto", new PathPlannerAuto("OurTestAuto"));

        // Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);

    
         SmartDashboard.putData("Medium Test Auto", autoChooser);


        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        //Hang to Joystick Command
        RobotContainer.mHangSub.setDefaultCommand(new hangStick());


        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-filterX.calculate(joystick.getLeftY()) * MaxSpeed * percentSlow) // Drive forward with negative Y (forward)
                    .withVelocityY(-filterY.calculate(joystick.getLeftX()) * MaxSpeed * percentSlow) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * percentSlow) // Drive counterclockwise with negative X (left) // This was negative before but I made it positive to make it turn the right direction
            )
        );

        //These are all for the drive the first two apply a brake and do something that I don't know
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        //These ones are for a slow mode on the use of the dPad buttons
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(Constants.dPadSlow).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-Constants.dPadSlow).withVelocityY(0))
        );

        joystick.pov(90).whileTrue(drivetrain.applyRequest(() -> 
            forwardStraight.withVelocityX(0).withVelocityY(-Constants.dPadSlow))
        );

        joystick.pov(270).whileTrue(drivetrain.applyRequest(() -> 
            forwardStraight.withVelocityX(0).withVelocityY(Constants.dPadSlow))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));



        //This slows down the drive on the press of a button
        joystick.leftBumper().onTrue(new InstantCommand(() -> slow()));

        


        // joystick.y().whileTrue(new SpitCoral());
        // joystick.x().whileTrue(new IntakeCoralCommand());
        codriver.y().whileTrue(new ManualElevatorUp());
        codriver.a().whileTrue(new ManualElevatorDown());
        //joystick.rightTrigger(0.5).whileTrue(new IntakeCoralCommand());
        //joystick.leftTrigger(0.5).whileTrue(new SpitCoral());
        codriver.rightTrigger(0.5).whileTrue(new ManualAlgaeArmDown());
        codriver.x().whileTrue(new IntakeAlgaeCommand());
        codriver.leftTrigger(0.5).whileTrue(new ManualAlgaeArmUp());
        codriver.b().whileTrue(new SpitAlgaeCommand());
        
        codriver.rightBumper().whileTrue(new IntakeCoralCommand());
        codriver.leftBumper().whileTrue(new SpitCoral());

        // codriver.rightBumper().whileTrue(new HangArmIn());
        // codriver.leftBumper().whileTrue(new HangArmOut());

        //codriver.leftBumper().whileTrue(new IntakeAlgaeAutomatic());


        //90 is right
        

        // codriver.a().whileTrue(new HangArmIn());
        // codriver.b().whileTrue(new HangArmOut());
        
        codriver.pov(180).onTrue(new CoralStationCommand());
        codriver.pov(270).onTrue(new CoralLvl1Command());
        codriver.pov(90).onTrue(new CoralLvl2Command());
        codriver.pov(0).onTrue(new CoralLvl3Command());


        //codriver.pov(0).onTrue(new CoralLvl3Command().andThen(new WristLvl2and3()));







        drivetrain.registerTelemetry(logger::telemeterize);

        
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        
        return autoChooser.getSelected();
    }

    public void slow(){
        if(percentSlow == 1){
            percentSlow = Constants.percentSlow;
        }

        else{
            percentSlow = 1;
        }
    }
}   
