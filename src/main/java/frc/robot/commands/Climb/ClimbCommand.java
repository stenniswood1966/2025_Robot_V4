// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommand extends Command {
  /** Creates a new ClimbCommand. */
  public ClimbCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.winchsubsystem, RobotContainer.algaerotatesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.algaerotatesubsystem.enablemotionmagic(constants.k_AlgaeRotateMMHandoff);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //bring climinator to handoff position 
    //shut the climinator regular motor off 
    //turn the winch on with mm
    //pray

    //System.out.println(RobotContainer.algaerotatesubsystem.getPosition());
    if (RobotContainer.winchsubsystem.getRotorPosition() > constants.k_WinchClimbHandoff){
      RobotContainer.algaerotatesubsystem.stop();      
    }

    if (RobotContainer.algaerotatesubsystem.getPosition() > constants.k_WinchClimbPosition) {
      RobotContainer.winchsubsystem.stop();
    } else {
      RobotContainer.winchsubsystem.set(constants.k_WinchClimbPower);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.algaerotatesubsystem.stop();
    RobotContainer.winchsubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
