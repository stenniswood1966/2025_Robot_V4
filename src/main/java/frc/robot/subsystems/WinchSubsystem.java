// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

public class WinchSubsystem extends SubsystemBase {
  static TalonFX motor1 = new TalonFX(13, "Flurb_Drive");
  static TalonFX motor2 = new TalonFX(0);//change me
  
  MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

  /** Creates a new WinchSubsystem. */
  public WinchSubsystem() {
    
    var fx_cfg = new TalonFXConfiguration(); //creates a default TalonFX configuration 
   
    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake; // set brake after tuning
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.407; //use absolute position of cancoder in tuner x
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.041;

    /* Configure current limits */
    MotionMagicConfigs mm = new MotionMagicConfigs(); //creates a default motion magic congiguration
    mm.MotionMagicCruiseVelocity = 150; // RotorVelocity per second
    mm.MotionMagicAcceleration = 150; // Take approximately 0.5 seconds to reach max vel Take approximately 0.2 seconds to reach max accel
    mm.MotionMagicJerk = 1000; //smooths out the transition from start/stop to cruise velocity
    fx_cfg.MotionMagic = mm;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 50; //output per unit of error in velocity (output/rps)
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

  public double getRotorPosition()
  {
    return Math.abs(motor1.getRotorPosition().getValueAsDouble());
  }

}
