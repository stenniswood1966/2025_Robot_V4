// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorSetPositionCommand extends Command {
  private double targetpos;

  /** Creates a new ElevatorL1Command. */
  public ElevatorSetPositionCommand(double targetpos) {
    this.targetpos = targetpos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.elevatorsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.elevatorsubsystem.enablemotionmagic(targetpos);
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
    if (!constants.k_ElevatorMMisMoving && (constants.k_ElevatorMMatPosition)) {
      return true;
    } else {
      return false;
    }
  }
}
