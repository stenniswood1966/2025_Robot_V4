// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

public class ShoulderSubsystem extends SubsystemBase {
  static TalonFX motor1 = new TalonFX(11, "Flurb_Elevator" );
  static CANcoder cancoder = new CANcoder (35, "Flurb_Elevator");

  MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

  /** Creates a new WristSubsystem. */
  public ShoulderSubsystem() {
    // Configure CANcoder to zero the magnet appropriately
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration(); // creates a default CANcoder configuration
    //cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    //cc_cfg MagnetSensor.MagnetOffset = cancoder.getAbsolutePosition().getValueAsDouble(); // sets absoulte value as mag offset
    cc_cfg.MagnetSensor.MagnetOffset = 0.0; // Change this to match required offset
    cancoder.getConfigurator().apply(cc_cfg); // apply configuration to cancoder

    var fx_cfg = new TalonFXConfiguration(); //creates a default TalonFX configuration
    fx_cfg.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
    fx_cfg.Feedback.RotorToSensorRatio = 100;

    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake; // set brake after tuning
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.60; //use absolute position of cancoder in tuner x was .38
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.11;

    /* Configure current limits */
    MotionMagicConfigs mm = new MotionMagicConfigs(); //creates a default motion magic congiguration
    mm.MotionMagicCruiseVelocity = 10; // RotorVelocity per second 10 - old
    mm.MotionMagicAcceleration = 15; // Take approximately 0.5 seconds to reach max vel Take approximately 0.2 seconds to reach max accel 5 - old
    mm.MotionMagicJerk = 1000; //smooths out the transition from start/stop to cruise velocity
    fx_cfg.MotionMagic = mm;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 100; //output per unit of error in velocity (output/rps)
    slot0.kI = 0.0; //output per unit of integrated error in velocity (output/rotation)
    slot0.kD = 0.0; //output per unit of error derivative in velocity (output/(rps/s))
    slot0.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kV = 0.0; //output per unit of requested velocity (output/rps)
    slot0.kS = 0.0; //output to overcome static friction (output)
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    fx_cfg.Slot0 = slot0; //adds the slot0 config to the motors configuration file

    motor1.getConfigurator().apply(fx_cfg, 0.050); //apply configuration to motor
  }

  @Override
  public void periodic() {
    if ((motor1.getVelocity().getValueAsDouble() == 0.0)){
      constants.k_ShoulderMMisMoving = false;
    }else {
      constants.k_ShoulderMMisMoving = true;
    }
  
    //System.out.println(Math.abs(motor1.getClosedLoopError().getValueAsDouble()));
    if (Math.abs(motor1.getClosedLoopError().getValueAsDouble()) < .0025) {
      //System.out.println("********************************************************************************");
      constants.k_ShoulderMMatPosition = true;
    }else {
      constants.k_ShoulderMMatPosition = false;
    }
  }

  public void set(Double speed)  {
    var motorRequest = new DutyCycleOut(speed); //Converts the double to DutyCycleOut
    motor1.setControl(motorRequest); // Requests the motor to move
  }

  public void stop(){
    motor1.set(0);
  }
  public void enablemotionmagic(double targetpos) {
    // periodic, run Motion Magi with slot 0 configs,
    motor1.setControl(mmReq.withPosition(targetpos).withSlot(0));
  }

  public void disablemotionmagic() {
    motor1.set(0);
  }
  
  public static void addWristModifier() {
    constants.k_WristModifyPosition = constants.k_WristModifyPosition + 0.001;
  }

  public static void subtractWristModifier() {
    constants.k_WristModifyPosition = constants.k_WristModifyPosition - 0.001;
  }

  public static void resetWristModifier() {
    constants.k_WristModifyPosition = 0.0;
  }

  //this method allows us to get the position of motor 1. This is important because we need to force the command that calls 
    //this to update the position 
  public double getPosition()
  {
    return Math.abs(motor1.getPosition().getValueAsDouble());
  }
}
