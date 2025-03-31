// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbPrepCommand extends Command {
  /** Creates a new ClimbPrepCommand. */
  public ClimbPrepCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.algaerotatesubsystem, RobotContainer.winchsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //release the foot
    //put the algae intake into prep position
    RobotContainer.winchsubsystem.set(constants.k_WinchReleasePower);
    RobotContainer.algaerotatesubsystem.enablemotionmagic(constants.k_AlgaeRotateMMClimbPrep);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.winchsubsystem.getRotorPosition() > constants.k_WinchRelease) {
      RobotContainer.winchsubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.winchsubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((RobotContainer.winchsubsystem.getRotorPosition() > constants.k_WinchRelease)) {
      return true;
    } else {
      return false;
    }
  }
}
