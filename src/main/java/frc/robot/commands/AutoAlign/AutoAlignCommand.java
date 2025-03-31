// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignCommand extends Command {
  double tx = 0;
  double ty = 0;
  double pipelineIndex;
  int tagOld;

  /* 
  //0,x is blue tags 1,x is red tags
  //practice field tag angles
    //our tags
  double[][] tagAngles = {{0,0,-34,0,0,-142,-90,0,38,156,90,0}, //-151
                          {0,0,-210,0,0,-142,-90,0,220,156,90,0}};
  */

  //port huron tags
  //double[][] tagAngles = {{0,0,0,0,0,0,210,270,330,30,90,145},
  //{0,0,0,0,0,0,210,270,330,30,90,145}};

  //real field tag angles
  
  double[][] tagAngles = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-29,273,212,151,90,31}, 
                          {0,0,0,0,0,0,210,270,330,30,90,145}};
  
  
  CommandSwerveDrivetrain _drivetrain;
  /** Creates a new AutoAlignCommand. */
  public AutoAlignCommand(CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    _drivetrain = drivetrain;
    addRequirements(RobotContainer.rangefindersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{6, 2, 10, 8, 9, 5});
    
    
    if (isAllianceRed() == 1) {
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{6,7,8,9,10,11}); //red alliance tags
      } else {
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{17,18,19,20,21,22}); //blue alliance tags
    }
    
    //setting the tag we can see to be the only tag that can be seen
    tagOld = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", new int[]{tagOld});
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("ReadyToEject", false);
    //grabbing the tx ty and pipeline index the limelight is currently giving out
    tx = LimelightHelpers.getTX("limelight");
    ty = LimelightHelpers.getTY("limelight");
    pipelineIndex = LimelightHelpers.getCurrentPipelineIndex("limelight");
    
    //grabbing the tag id for the rotation angle 
    int tagID = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1);

    //converting tagID to tag angle for the heading of the robot
    //System.out.println(tagID);
    if (tagID != -1) {
      //System.out.println(tagAngles[isAllianceRed()][tagID]);
      constants.k_steering_target = new Rotation2d(Math.toRadians(tagAngles[isAllianceRed()][tagID]));
    } if (tagID == -1) {
      if (RobotContainer.rangefindersubsystem.getRange() > .4 || RobotContainer.rangefindersubsystem.getRange() == 0.0) {
        constants.k_steering_target = null;
      }
    }
    //System.out.println(constants.k_steering_target);

    //throwing tx into a constant
    constants.k_x_target = tx;    

    //using the range finder to determine if we should be apply y power or not
    //need to check if the rangefinder is throwing out a zero cause that is what it throws out when it cannot see anything
    if (RobotContainer.rangefindersubsystem.getRange() <= constants.k_rangeFinderMin && RobotContainer.rangefindersubsystem.getRange() != 0.0) {//tune me
      constants.k_y_power = 0;
    } else {
      constants.k_y_power = .3; //treat as a joystick 
    }
    //System.out.println(constants.k_y_power);

  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //this stuff sets the limelight to able to see all the tag again
    SmartDashboard.putBoolean("ReadyToEject", true);
    //LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{6, 2, 10, 8, 9, 5});

    
     //when comp comes un comment me
    if (isAllianceRed() == 1) {
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{6,7,8,9,10,11}); //red alliance tags
    } else {
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{17,18,19,20,21,22}); //blue alliance tags
    }
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //checking to see if the range finder see something and is below our min range
    if (RobotContainer.rangefindersubsystem.getRange() <= constants.k_rangeFinderMin && RobotContainer.rangefindersubsystem.getRange() > .15) {//tune me
      //System.out.println("aaaaaaaaaaaaaaaaaaa");
      return true;
    } else {
      return false;
    }
  }

  //getting the alliance that is currently selected in the driver station and returning it as an integer
  private int isAllianceRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Red) {
      return 1;
    } else {
      return 0;
    }
  }
}
