// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShoulderElevator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetShoulderElevatorPosition extends Command {
  private double targetposshoulder;
  private double targetposelevator;

  /** Creates a new SetShoulderElevatorPosition. */
  public SetShoulderElevatorPosition() {
    this.targetposshoulder = constants.k_ShoulderMMHome;
    this.targetposelevator = constants.k_ElevatorMMHome;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public SetShoulderElevatorPosition(double targetposshoulder, double targetposelevator) {
    this.targetposshoulder = targetposshoulder;
    this.targetposelevator = targetposelevator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //setting the constants that are used to control the shoulder and elevators position
    constants.k_ShoulderPos = targetposshoulder;
    constants.k_ElevatorPos = targetposelevator;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
