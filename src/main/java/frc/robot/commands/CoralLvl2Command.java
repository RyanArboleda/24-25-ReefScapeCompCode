// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.elevatorSetpoints;
import frc.robot.subsystems.coralSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralLvl2Command extends Command {

  private double initial;
  /** Creates a new CoralLvl2Command. */
  public CoralLvl2Command() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mCoralSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.initial = RobotContainer.mCoralSub.getElevatorRelEncoder();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.mCoralSub.moveToSetpointLevelTwo();
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
    // double height = RobotContainer.mCoralSub.getElevatorRelEncoder();
    // //lvl2 is 6.08
    // System.out.println("Level 2 height: " + elevatorSetpoints.lvl2);
    // System.out.println("Initial Height: " + this.initial);
    
    
    // if (this.initial < elevatorSetpoints.lvl2) { //if initial height is below lvl 2
    //   System.out.println("Entered First Outer if");
    //   if(height >= elevatorSetpoints.lvl2 - 0.1){ //if current height greater than or equaled to lvl 2 return true
    //     System.out.println("Entered first if");
    //     return true;
    //   }
    //   else return false;
    // }


    // else if (this.initial > elevatorSetpoints.lvl2) { //if initial height is above lvl 2
    //   System.out.println("Entered Second outer if"); // enteres here
    //   if (height <= elevatorSetpoints.lvl2 + 0.1) { //if current height is below lvl 2 return true
    //     System.out.println("Enetered Second if"); // never reaches here
    //     return true;
    //   }
    //   else return false;
    // }
    
    // else{
    // return false;
    // }
    if(RobotContainer.mCoralSub.getElevatorRelEncoder() >= Constants.elevatorSetpoints.lvl2){
      return true;
    }
    else{
    return false;
  }
}
}
