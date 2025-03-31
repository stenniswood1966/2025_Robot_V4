// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShoulderElevator;

import javax.print.event.PrintJobListener;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShoulderElevatorCommand extends Command {
  private double targetposelevatorOld;
  private boolean intakeBool;
  private boolean homeBool;

  /** Creates a new L2Command. */
  public ShoulderElevatorCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shouldersubsystem, RobotContainer.elevatorsubsystem, RobotContainer.effectorsubsystem);
    this.intakeBool = false;
    this.homeBool = false;
  }

  public ShoulderElevatorCommand(boolean intakeBool) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shouldersubsystem, RobotContainer.elevatorsubsystem, RobotContainer.effectorsubsystem);
    this.intakeBool = intakeBool;
    this.homeBool = false;
  }

  public ShoulderElevatorCommand(boolean intakeBool, boolean homeBool) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shouldersubsystem, RobotContainer.elevatorsubsystem, RobotContainer.effectorsubsystem);
    this.intakeBool = intakeBool;
    this.homeBool = homeBool;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //setting the old target position to zero to force the arm to home itself on the first run through
    this.targetposelevatorOld = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //if homeBool is true use the home setpoints instead of the regular setpoints if it is false use the regular setpoints 
    //this allows us to have the end effector remember the selected level until the operator changes it
    if (homeBool) {
      //if new elevator position move the arm to the home position
      if (constants.k_ElevatorMMHome != this.targetposelevatorOld) {
        //enabling motion magic on the shoulder with the position contained within the constants file for the home position
        RobotContainer.shouldersubsystem.enablemotionmagic(constants.k_ShoulderMMHome);

        //updating the targetposelevatorOld to the new elevator position so this if statement only runs once per elevator update
        this.targetposelevatorOld = constants.k_ElevatorMMHome;

        //System.out.println("trigger1");
      }
      

      //if the shoulder is at the home position tell the elevator to move to its new position
      //this if statement must do the difference calculation here to force everything to update allowing us to grab the most recent positions of the 
        //shoulder. 
      //System.out.println(Math.abs(constants.k_ShoulderMMHome - RobotContainer.shouldersubsystem.getPosition()));
      if (Math.abs(constants.k_ShoulderMMHome - RobotContainer.shouldersubsystem.getPosition()) < constants.k_ShoulderRange) { 
        //enabling motion magic on the elevator with the position contained within the constants file for currently selected position
        RobotContainer.elevatorsubsystem.enablemotionmagic(constants.k_ElevatorMMHome);

        //System.out.println("trigger2");
      }  

      //if elevator and shoulder is at the assigned position tell the shoulder to move to its new position
      //this if statement must do the difference calculation here to force everything to update allowing us to grab the most recent positions of the 
        //shoulder and elevator
      if (Math.abs(constants.k_ElevatorMMHome - RobotContainer.elevatorsubsystem.getPosition()) < constants.k_ElevatorRange &&
        (Math.abs(constants.k_ShoulderMMHome - RobotContainer.shouldersubsystem.getPosition()) < constants.k_ShoulderRange)) {
        //enabling motion magic on the elevator with the position contained within the constants file for currently selected position
        RobotContainer.shouldersubsystem.enablemotionmagic(constants.k_ShoulderMMHome);

        //System.out.println("trigger3");
      }
    } else {
      if (intakeBool) {
        //if new elevator position move the arm to the home position
        if (constants.k_ElevatorMMIntake != this.targetposelevatorOld) {
          //enabling motion magic on the shoulder with the position contained within the constants file for the home position
          RobotContainer.shouldersubsystem.enablemotionmagic(constants.k_ShoulderMMHome);

          //updating the targetposelevatorOld to the new elevator position so this if statement only runs once per elevator update
          this.targetposelevatorOld = constants.k_ElevatorMMIntake;

          //System.out.println("trigger1");
        }
        

        //if the shoulder is at the home position tell the elevator to move to its new position
        //this if statement must do the difference calculation here to force everything to update allowing us to grab the most recent positions of the 
          //shoulder. 
        if (Math.abs(constants.k_ShoulderMMHome - RobotContainer.shouldersubsystem.getPosition()) < constants.k_ShoulderRange) { 
          //enabling motion magic on the elevator with the position contained within the constants file for currently selected position
          RobotContainer.elevatorsubsystem.enablemotionmagic(constants.k_ElevatorMMIntake);

          //System.out.println("trigger2");
        }  

        //if elevator and shoulder is at the assigned position tell the shoulder to move to its new position
        //this if statement must do the difference calculation here to force everything to update allowing us to grab the most recent positions of the 
          //shoulder and elevator
        if (Math.abs(constants.k_ElevatorMMIntake - RobotContainer.elevatorsubsystem.getPosition()) < constants.k_ElevatorRange &&
          (Math.abs(constants.k_ShoulderMMHome - RobotContainer.shouldersubsystem.getPosition()) < constants.k_ShoulderRange)) {
          //enabling motion magic on the elevator with the position contained within the constants file for currently selected position
          RobotContainer.shouldersubsystem.enablemotionmagic(constants.k_ShoulderMMIntake);

          //System.out.println("trigger3");
        }
      } else {
        //if new elevator position move the arm to the home position
        if (constants.k_ElevatorPos != this.targetposelevatorOld) {
          //enabling motion magic on the shoulder with the position contained within the constants file for the home position
          RobotContainer.shouldersubsystem.enablemotionmagic(constants.k_ShoulderMMHome);

          //updating the targetposelevatorOld to the new elevator position so this if statement only runs once per elevator update
          this.targetposelevatorOld = constants.k_ElevatorPos;

          //System.out.println("trigger1");
        }
        

        //if the shoulder is at the home position tell the elevator to move to its new position
        //this if statement must do the difference calculation here to force everything to update allowing us to grab the most recent positions of the 
          //shoulder. 
        if (Math.abs(constants.k_ShoulderMMHome - RobotContainer.shouldersubsystem.getPosition()) < constants.k_ShoulderRange) { 
          //enabling motion magic on the elevator with the position contained within the constants file for currently selected position
          RobotContainer.elevatorsubsystem.enablemotionmagic(constants.k_ElevatorPos);

          //System.out.println("trigger2");
        }  

        //if elevator and shoulder is at the assigned position tell the shoulder to move to its new position
        //this if statement must do the difference calculation here to force everything to update allowing us to grab the most recent positions of the 
          //shoulder and elevator
        if (Math.abs(constants.k_ElevatorPos - RobotContainer.elevatorsubsystem.getPosition()) < constants.k_ElevatorRange &&
          (Math.abs(constants.k_ShoulderMMHome - RobotContainer.shouldersubsystem.getPosition()) < constants.k_ShoulderRange)) {
          //enabling motion magic on the elevator with the position contained within the constants file for currently selected position
          RobotContainer.shouldersubsystem.enablemotionmagic(constants.k_ShoulderPos);

          //System.out.println("trigger3");
        }
      }
    }
    
    //checking if the intakeBool boolean is true and if it is turning the end effector on to intake mode
    if (this.intakeBool) {
      RobotContainer.effectorsubsystem.effectorintake();

      //System.out.println("trigger4");
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (homeBool) {
      return Math.abs(constants.k_ElevatorMMHome - RobotContainer.elevatorsubsystem.getPosition()) < constants.k_ElevatorRange &&
        (Math.abs(constants.k_ShoulderMMHome - RobotContainer.shouldersubsystem.getPosition()) < constants.k_ShoulderRange);
    } else {
      if (intakeBool) {
        //checking to see if the Elevator and Shoulder are at the positions that we want them to be at
        return (Math.abs(constants.k_ElevatorMMIntake - RobotContainer.elevatorsubsystem.getPosition()) < constants.k_ElevatorRange && 
        (Math.abs(Math.abs(constants.k_ShoulderMMIntake) - Math.abs(RobotContainer.shouldersubsystem.getPosition())) < constants.k_ShoulderRange));
      } else {
        //checking to see if the Elevator and Shoulder are at the positions that we want them to be at
        return (Math.abs(constants.k_ElevatorPos - RobotContainer.elevatorsubsystem.getPosition()) < constants.k_ElevatorRange && 
        (Math.abs(Math.abs(constants.k_ShoulderPos) - Math.abs(RobotContainer.shouldersubsystem.getPosition())) < constants.k_ShoulderRange));
      }
    }
   
  }
}
