// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralLvl3Command extends Command {
  Timer timer;
  /** Creates a new CoralLvl3Command. */
  public CoralLvl3Command() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mCoralSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.mCoralSub.moveToSetpointLevelThree();
    //RobotContainer.mCoralSub.moveToSetpointWristLevelTwoAndThree();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.mCoralSub.elevatorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.mCoralSub.getElevatorRelEncoder() >= Constants.elevatorSetpoints.lvl3 - 0.5){
      return true;
    }
        return false;
  }
}
