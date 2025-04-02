// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class visionSub extends SubsystemBase {
  /** Creates a new visionSub. */
  private RawFiducial[] fiducials;
  //private final LEDSub
  public visionSub() {
    config();
  }

  public static class NoSuchTargetException extends RuntimeException{// no "fiducial" found
    public NoSuchTargetException(String message){
      super(message);
    }

  }
  public void config(){
    LimelightHelpers.setCameraPose_RobotSpace(Constants.VisionConstants.LimeLightName, 0, 0, 0, 0, 0, 0);
    LimelightHelpers.SetFiducialIDFiltersOverride(Constants.VisionConstants.LimeLightName, new int[] {0, 1, 3, 5, 6, 8, 9, 10, 11, 12});

    SmartDashboard.putNumber("rotP", 0);
    SmartDashboard.putNumber("rotD", 0);
  }

  public boolean isAligned(){
    if (Math.abs(getTx()-Constants.VisionConstants.reefAngle)<Constants.VisionConstants.reefTolerance){return true;}//left
    if (Math.abs(getTx()+Constants.VisionConstants.reefAngle)<Constants.VisionConstants.reefTolerance){return true;}//right
    return false;
  }

  public double getTx(){
    return LimelightHelpers.getTX(Constants.VisionConstants.LimeLightName);
  }

  public Angle getTxAngle(){
    return Angle.ofBaseUnits(LimelightHelpers.getTX(Constants.VisionConstants.LimeLightName), Degrees);
  }
  
  public double getTy(){
    return LimelightHelpers.getTY(Constants.VisionConstants.LimeLightName);
  }

  public Angle getTyAngle(){
    return Angle.ofBaseUnits(LimelightHelpers.getTY(Constants.VisionConstants.LimeLightName), Degrees);
  }

  public double getTa(){
    return LimelightHelpers.getTA(Constants.VisionConstants.LimeLightName);
  }

  public boolean getTv(){
    return LimelightHelpers.getTV(Constants.VisionConstants.LimeLightName);
  }

  public double getClosestTX(){
    return getClosestFiducial().txnc;
  }
  public double getClosestTY(){
    return getClosestFiducial().tync;
  }
  public double getClosestTA(){
    return getClosestFiducial().ta;
  }

  public double getID_TX(int ID){
    return getFiducialWithId(ID).txnc;
  }
  public double getID_TY(int ID){
    return getFiducialWithId(ID).tync;
  }
 //Linear searcgh by id
 public RawFiducial getFiducialWithId(int id) {
  
  for (RawFiducial fiducial : fiducials) {
      if (fiducial.id == id) {
          return fiducial;
      }
  }
  throw new NoSuchTargetException("Can't find ID: " + id);
}

public RawFiducial getFiducialWithId(int id, boolean verbose) {//Debug
StringBuilder availableIds = new StringBuilder();

for (RawFiducial fiducial : fiducials) {
    if (availableIds.length() > 0) {
        availableIds.append(", ");
    } //Error reporting
    availableIds.append(fiducial.id);
    
    if (fiducial.id == id) {
        return fiducial;
    }
}
throw new NoSuchTargetException("Cannot find: " + id + ". IN view:: " + availableIds.toString());
}

  public RawFiducial getClosestFiducial() {
    if (fiducials == null || fiducials.length == 0) {
        throw new NoSuchTargetException("No fiducials found.");
    }

    RawFiducial closest = fiducials[0];
    double minDistance = closest.ta;
    //Linear search for close
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.ta > minDistance) {
            closest = fiducial;
            minDistance = fiducial.ta;
        }
    }
    return closest;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    fiducials = LimelightHelpers.getRawFiducials(Constants.VisionConstants.LimeLightName);
    SmartDashboard.putBoolean("aligned", isAligned());

    

    
    
    
  }
}
