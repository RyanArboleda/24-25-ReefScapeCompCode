// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class hangStick extends Command {
  /** Creates a new hangStick. */
  CommandXboxController codriver = new CommandXboxController(Constants.codriver);
  public hangStick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mHangSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(codriver.getLeftY() > 0.2){
      RobotContainer.mHangSub.hangSetPower(1);
    }
    else if(codriver.getLeftY() < -0.2){
      RobotContainer.mHangSub.hangSetPower(-0.5);
    }
    else{
    RobotContainer.mHangSub.hangSetPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.mHangSub.hangStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
