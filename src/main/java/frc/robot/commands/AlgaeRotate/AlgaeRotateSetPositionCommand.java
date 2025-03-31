// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeRotate;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeRotateSetPositionCommand extends Command {
  private double targetpos;

  /** Creates a new ElevatorL1Command. */
  public AlgaeRotateSetPositionCommand(double targetpos) {
    this.targetpos = targetpos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.algaerotatesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.algaerotatesubsystem.enablemotionmagic(targetpos);
    //System.out.println(targetpos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    if (Math.abs(RobotContainer.algaerotatesubsystem.getPosition() - targetpos) < constants.k_AlgaeRotateRange){//tune me
      return true;
    } else {
      return false;
    }
  }
}
