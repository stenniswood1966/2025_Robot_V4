// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.RobotContainer;
import frc.robot.constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  static TalonFX motor1 = new TalonFX (16, "Flurb_Elevator");
  static TalonFX motor2 = new TalonFX(17, "Flurb_Elevator");

  MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    //configure the TalonFX motors
    var fx_cfg = new TalonFXConfiguration(); //creates a default TalonFX configuration

    //configuration for motor2
    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 111;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 5.0;

    //set motor2 as a strict follower
    motor2.setControl(new StrictFollower(motor1.getDeviceID()));
    //apply configuration to motor 2
    motor2.getConfigurator().apply(fx_cfg, 0.050); //apply configuration to motor

   //configuration for motor1
   fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
   fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
   fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
   fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
   fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 111;
   fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 5.0;

   // set slot 0 gains
   var slot0Configs = fx_cfg.Slot0;
   slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
   slot0Configs.kV = 0.2; // A velocity target of 1 rps results in 0.12 V output
   slot0Configs.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
   slot0Configs.kP = 12; // A position error of 1.5 rotation results in 12 V output WAS 10
   slot0Configs.kI = 0; // no output for integrated error
   slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
   slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

   //set Motion Magic Settings
  var motionMagicConfigs = fx_cfg.MotionMagic;
  motionMagicConfigs.MotionMagicCruiseVelocity = 200; // Target cruise velocity of 90 rps 150 - old
  motionMagicConfigs.MotionMagicAcceleration = 200; // Target acceleration of 160 rps/s (0.5 seconds) 150 - old
  motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds); 1100 - old

  motor1.getConfigurator().apply(fx_cfg, 0.050); //apply configuration to motor

  //fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  //motor2.getConfigurator().apply(fx_cfg, 0.050); // apply configuration to motor

  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if ((motor1.getVelocity().getValueAsDouble() == 0.0)){
      constants.k_ElevatorMMisMoving = false;
    }else {
      constants.k_ElevatorMMisMoving = true;
    }

    //System.out.println(Math.abs(motor1.getClosedLoopError().getValueAsDouble()));
    if (Math.abs(motor1.getClosedLoopError().getValueAsDouble()) < .04) {
      constants.k_ElevatorMMatPosition = true;
    }else {
      constants.k_ElevatorMMatPosition = false;
    }

    constants.k_ElevatorCurrentPosition = motor1.getPosition().getValueAsDouble();
    if (motor1.getPosition().getValueAsDouble() < constants.k_ElevatorDownRange) { 
      constants.k_ElevatorIsDown = true;
    }else {
      constants.k_ElevatorIsDown = false;
    }
  }

  public void set(Double speed){
    var motorRequest = new DutyCycleOut(speed); //Converts the double to DutyCycleOut
    motor1.setControl(motorRequest); // Requests the motor to move

    //motor2.setControl(motorRequest); // Requests the motor to move
  }

  public void enablemotionmagic(double targetpos) {
    // periodic, run Motion Magi with slot 0 configs,
    motor1.setControl(mmReq.withPosition(targetpos).withSlot(0));
  }

  public void stop()
  {
    motor1.set(0);

    motor2.set(0);
  }

  //this method allows us to get the position of motor 1. This is important because we need to force the command that calls 
    //this to update the position 
  public double getPosition()
  {
    return Math.abs(motor1.getPosition().getValueAsDouble());
  }
} 
