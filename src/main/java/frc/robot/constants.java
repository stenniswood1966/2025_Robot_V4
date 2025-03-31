// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class constants {
    public static double k_WristModifyPosition;

    public static Rotation2d k_steering_target;
    public static double k_x_target = 0;
    public static double k_y_power = 0;
    public static double k_rangeFinderMin = .225;


    public static boolean k_CoralisReady = false;

    public static boolean k_ShoulderMMisMoving = false;
    public static boolean k_ShoulderMMatPosition = false;
    public static double k_ShoulderRange = .020;

    public static boolean k_ElevatorMMisMoving = false;
    public static boolean k_ElevatorMMatPosition = false;
    public static double k_ElevatorRange = .038;//.038
    public static boolean k_ElevatorIsDown = true; //true is down false is up
    public static double k_ElevatorDownRange = 10;
    public static double k_ElevatorCurrentPosition = 0;

    //shoulder and elevator set points
    public static double k_ShoulderMMHomeLow = .107;
    public static double k_ElevatorMMHomeLow = 5;

    public static double k_ShoulderMMHome = .107;//.16
    public static double k_ElevatorMMHome = 32;//5

    public static double k_ShoulderMML1 = .60;
    public static double k_ElevatorMML1 = 43;
    
    public static double k_ShoulderMML2 = .035;
    public static double k_ElevatorMML2 = 32;

    public static double k_ShoulderMML3 = .035;
    public static double k_ElevatorMML3 = 61;

    public static double k_ShoulderMML4 = .010;//.025
    public static double k_ElevatorMML4 = 110;//108

    public static double k_ShoulderMMIntake = .55;//.5
    public static double k_ElevatorMMIntake = 60;//50
    
    public static double k_ShoulderMMIntakeAuton = .55;
    public static double k_ElevatorMMIntakeAuton = 60;

    public static double k_ShoulderMMClimb = .38;
    public static double k_ElevatorMMClimb = 5; 


    //the global constants the hold the currently selected position
    public static double k_ShoulderPos = constants.k_ShoulderMMHome;
    public static double k_ElevatorPos = constants.k_ElevatorMMHome;

    //algae rotate values
    public static double k_AlgaeRotateRange = .05;

    public static double k_AlgaeRotateMMHome = 0;//0

    public static double k_AlgaeRotateMMPickup = -3.5;

    public static double k_AlgaeRotateMMClimbPrep = -6.4;

    public static double k_AlgaeRotateMMHandoff = -3.07;

    //Winch values
    public static double k_WinchRelease = 24;
    public static double k_WinchReleasePower = .2;

    public static double k_WinchClimbHandoff = 50;
    
    public static double k_WinchClimbPower = .25;

    public static double k_WinchClimbPosition = 1.6;
}
