// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RangeFinderSubsystem extends SubsystemBase {
  public static CANrange canRange1 = new CANrange(42, "Flurb_Drive");

  /** Creates a new RangeFinder. */
  public RangeFinderSubsystem() {
    //configuring the canrange
    var cr_cfg = new CANrangeConfiguration();
    cr_cfg.ProximityParams.ProximityThreshold = .09; //is in meters
    cr_cfg.ProximityParams.ProximityHysteresis = .001;
    
    //applying the canrange config to the canrange
    canRange1.getConfigurator().apply(cr_cfg);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //this method is used to get the canrange distance to other commands and subsystems
  public double getRange(){
    //System.out.println(canRange1.getDistance().getValueAsDouble());
    return canRange1.getDistance().getValueAsDouble();
  }
}
