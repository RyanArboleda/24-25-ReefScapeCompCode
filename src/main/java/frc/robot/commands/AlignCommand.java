// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.visionSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCommand extends Command {

  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private visionSub m_Vision;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain m_Swerve;

  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  /** Creates a new alignCommand. */
  public AlignCommand(CommandSwerveDrivetrain swerve, visionSub limelight, boolean isRightScore) {
    xController = new PIDController(VisionConstants.reefAlignX, 0, 0);  // Vertical movement
    yController = new PIDController(VisionConstants.reefAlignY, 0, 0);  // Horitontal movement
    rotController = new PIDController(VisionConstants.reefAlignRot, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.m_Vision = limelight;
    this.m_Swerve = swerve;
    addRequirements(swerve, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(VisionConstants.reefAlignRotSetpos);
    rotController.setTolerance(VisionConstants.reefAlignRotTolerance);

    xController.setSetpoint(VisionConstants.reefAlignXSetpos);
    xController.setTolerance(VisionConstants.reefAlignTXolerance);

    yController.setSetpoint(isRightScore ? VisionConstants.reefAlignYSetpos : -VisionConstants.reefAlignYSetpos);
    yController.setTolerance(VisionConstants.reefAlignYTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Vision.getTv()) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);
      
      double xSpeed = xController.calculate(postions[2]);
      //SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = rotController.calculate(postions[4]);
      m_Swerve.setControl(alignRequest.withVelocityX(yController.getError() < VisionConstants.reefAlignYTolerance ? xSpeed : 0).withRotationalRate(rotValue).withVelocityY(ySpeed));

      m_Swerve.setControl(alignRequest.withVelocityX(yController.getError() < VisionConstants.reefAlignYTolerance ? 0 : 0).withRotationalRate(rotValue).withVelocityY(ySpeed));


      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ){//||
          //!xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      m_Swerve.setControl(idleRequest);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.setControl(idleRequest);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(VisionConstants.waitTime) ||
        stopTimer.hasElapsed(VisionConstants.validationTime);  
    }
}
