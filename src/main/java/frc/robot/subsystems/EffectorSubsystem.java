// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

public class EffectorSubsystem extends SubsystemBase {
  public static TalonFX motor1 = new TalonFX(12, "Flurb_Elevator");
  public static CANrange canRange1 = new CANrange(41, "Flurb_Elevator");

  // class member variable
  final VelocityVoltage m_Velocity = new VelocityVoltage(0);
  /** Creates a new EffectorSubsystem. */
  public EffectorSubsystem() {
    var cr_cfg = new CANrangeConfiguration();
    cr_cfg.ProximityParams.ProximityThreshold = .09; //is in meters
    cr_cfg.ProximityParams.ProximityHysteresis = .001;

    canRange1.getConfigurator().apply(cr_cfg);

    var fx_cfg = new TalonFXConfiguration(); // creates a default TalonFX configuration
    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    fx_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 152.0;
    fx_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    fx_cfg.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANrange;
    fx_cfg.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 41;
    fx_cfg.HardwareLimitSwitch.ForwardLimitEnable = true;

    motor1.getConfigurator().apply(fx_cfg, 0.050);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //this mess sets the global constant of k_CoralisReady to true when a coral is triggering the forward limit switch on the effector motor
    var forwardLimit = motor1.getForwardLimit();
    if (forwardLimit.getValue() == ForwardLimitValue.ClosedToGround) {
      // do action when forward limit is closed
      constants.k_CoralisReady = true;
    } else {
      constants.k_CoralisReady = false;
    }
  }

    public void effectorintake() {
      motor1.set(0.5);
    }

    public void effectorouttake(){
      if (constants.k_ElevatorPos == constants.k_ElevatorMML1) {
        motor1.set(-0.4);//tune me
      } else {
        motor1.set(-0.5);
      }
    }

    public void set(Double speed) {
      var motorRequest = new DutyCycleOut(speed); //Converts the double to DutyCycleOut
      motor1.setControl(motorRequest); // Requests the motor to move
    }

    public void stop() {
      motor1.set(0.0);
    }
}
