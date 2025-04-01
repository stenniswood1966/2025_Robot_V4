// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AlgaeIntake.*;
import frc.robot.commands.AlgaeRotate.*;
import frc.robot.commands.AutoAlign.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.EndEffector.*;
import frc.robot.commands.Shoulder.*;
import frc.robot.commands.Climb.*;
import frc.robot.commands.ShoulderElevator.*;
import frc.robot.commands.Winch.*;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import pabeles.concurrency.IntOperatorTask.Min;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double MinSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 3; // min speed used during go slow

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //XK-80 HID keypad
    private final XboxController m_operator1Controller = new XboxController(1);
    private JoystickButton Button_1 = new JoystickButton(m_operator1Controller, 1);
    private JoystickButton Button_2 = new JoystickButton(m_operator1Controller, 2);
    private JoystickButton Button_3 = new JoystickButton(m_operator1Controller, 3);
    private JoystickButton Button_4 = new JoystickButton(m_operator1Controller, 4);
    private JoystickButton Button_5 = new JoystickButton(m_operator1Controller, 5);
    private JoystickButton Button_6 = new JoystickButton(m_operator1Controller, 6);
    private JoystickButton Button_7 = new JoystickButton(m_operator1Controller, 7);
    private JoystickButton Button_8 = new JoystickButton(m_operator1Controller, 8);
    private JoystickButton Button_9 = new JoystickButton(m_operator1Controller, 9);
    private JoystickButton Button_10 = new JoystickButton(m_operator1Controller, 10);
    private JoystickButton Button_11 = new JoystickButton(m_operator1Controller, 11);
    private JoystickButton Button_12 = new JoystickButton(m_operator1Controller, 12);
    private JoystickButton Button_13 = new JoystickButton(m_operator1Controller, 13);
    private JoystickButton Button_14 = new JoystickButton(m_operator1Controller, 14);
    private JoystickButton Button_15 = new JoystickButton(m_operator1Controller, 15);
    private JoystickButton Button_16 = new JoystickButton(m_operator1Controller, 16);
    private JoystickButton Button_17 = new JoystickButton(m_operator1Controller, 17);
    private JoystickButton Button_18 = new JoystickButton(m_operator1Controller, 18);
    private JoystickButton Button_19 = new JoystickButton(m_operator1Controller, 19);
    private JoystickButton Button_20 = new JoystickButton(m_operator1Controller, 20);


    //subsystems used
    public static AlgaeIntakeSubsystem algaeintakesubsystem = new AlgaeIntakeSubsystem();
    public static EffectorSubsystem effectorsubsystem = new EffectorSubsystem();
    public static ShoulderSubsystem shouldersubsystem = new ShoulderSubsystem();
    public static ElevatorSubsystem elevatorsubsystem = new ElevatorSubsystem();
    public static AlgaeRotateSubsystem algaerotatesubsystem = new AlgaeRotateSubsystem();
    public static LEDSubsystem ledsubsystem = new LEDSubsystem();
    public static RangeFinderSubsystem rangefindersubsystem = new RangeFinderSubsystem();
    public static WinchSubsystem winchsubsystem = new WinchSubsystem();

    private final SwerveRequest.FieldCentricFacingAngle fieldcentricfacingangle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * .1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final SwerveRequest.RobotCentricFacingAngle robotcentricfacingangle = new SwerveRequest.RobotCentricFacingAngle()
        .withDeadband(MaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController joystick = new CommandXboxController(0);

    //controller 3 used to command manual stuff
    public static CommandXboxController joystick2 = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        namedcommands(); //pathplanner namedcommands

        autoChooser = AutoBuilder.buildAutoChooser("Exist");
        SmartDashboard.putData("Auto Mode", autoChooser);

        /* 
        //this mess makes sure that the limelights are in a state in which they can actually see the correct tags
            //since i believe that the settings stay through power cycle. 
            //it does it for each pipeline to prevent weirdness
        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{6, 2, 10, 8, 9, 5});
        LimelightHelpers.setPipelineIndex("limelight", 1);
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{6, 2, 10, 8, 9, 5});
        LimelightHelpers.setPipelineIndex("limelight", 0);
        */

         
        //when comp comes un comment me
        if (isAllianceRed() == 1) {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{6,7,8,9,10,11}); //red alliance tags
        } else {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{17,18,19,20,21,22}); //blue alliance tags
        }
        
        LimelightHelpers.setPipelineIndex("limelight", 1);
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{6, 2, 10, 8, 9, 5});
        //when comp comes un comment me
        if (isAllianceRed() == 1) {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{6,7,8,9,10,11}); //red alliance tags
        } else {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int[]{17,18,19,20,21,22}); //blue alliance tags
        }
        
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * getCalculatedMaxSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * getCalculatedMaxSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //joystick.y().whileTrue(drivetrain.applyRequest(() -> brake));

        fieldcentricfacingangle.HeadingController = new PhoenixPIDController(10, 0, 0);
        robotcentricfacingangle.HeadingController = new PhoenixPIDController(10,0,0);

        joystick.a().whileTrue(drivetrain.applyRequest(() -> robotcentricfacingangle
            .withVelocityY(constants.k_y_power * MinSpeed)
            .withVelocityX(constants.k_x_target * .05)
            .withTargetDirection(constants.k_steering_target) //this would be the angle to line up with
            ).ignoringDisable(true))
            .whileTrue(new AutoAlignCommand(drivetrain).repeatedly()
        );

        //joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        //));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-.5))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(.5))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        // reset the field-centric heading on start button press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        //staging the end effector to the selected level with auto align
        joystick.rightBumper().whileTrue(new ShoulderElevatorCommand())
            .whileTrue(drivetrain.applyRequest(() -> robotcentricfacingangle
                .withVelocityY(constants.k_y_power * MinSpeed)
                .withVelocityX(constants.k_x_target * .05)
                .withTargetDirection(constants.k_steering_target) //this would be the angle to line up with
                ).ignoringDisable(true))
                .whileTrue(new AutoAlignCommand(drivetrain).repeatedly())
            .onFalse(new ShoulderElevatorCommand(false, true));
        
        //staging the end effector on the selected level without auto align
        joystick.y().whileTrue(new ShoulderElevatorCommand())
        .onFalse(new ShoulderElevatorCommand(false, true));
        
        //putting the end effector into the intake position 
        joystick.leftBumper().whileTrue(new ShoulderElevatorCommand(true))
            .onFalse(new ShoulderElevatorCommand(false, true));

        //outtaking the coral hopefully onto the reef
        joystick.rightTrigger().whileTrue(new OuttakeCommand());

         
        //joystick.leftTrigger().whileTrue(new AlgaeRotateSetPositionCommand(constants.k_AlgaeRotateMMPickup)
           // .alongWith(new AlgaeIntakeIntakeCommand())).onFalse(new AlgaeRotateSetPositionCommand(constants.k_AlgaeRotateMMHome));
        joystick.x().whileTrue(new AlgaeIntakeOuttakeCommand());
        

    //assigning the  operator controls

        
        //Places the end effector in the low home position
        Button_1.whileTrue(new SetShoulderElevatorPosition(constants.k_ShoulderMMHomeLow, constants.k_ElevatorMMHomeLow)
            .andThen(new ShoulderElevatorCommand())); //Home 

        //these buttons control where the end effector ends up when the driver stages the bot and what side of the april tag we align to
        //left pipeline assign
        Button_2.whileTrue(new SetShoulderElevatorPosition(constants.k_ShoulderMML1, constants.k_ElevatorMML1)
            .alongWith(new ChangePiplineCommand(2))); //L1
        Button_3.whileTrue(new SetShoulderElevatorPosition(constants.k_ShoulderMML2, constants.k_ElevatorMML2)
            .alongWith(new ChangePiplineCommand(0))); //L2
        Button_4.whileTrue(new SetShoulderElevatorPosition(constants.k_ShoulderMML3, constants.k_ElevatorMML3)
            .alongWith(new ChangePiplineCommand(0))); //L3
        Button_5.whileTrue(new SetShoulderElevatorPosition(constants.k_ShoulderMML4, constants.k_ElevatorMML4)
            .alongWith(new ChangePiplineCommand(0))); //L4
        //right pipeline assign
        Button_6.whileTrue(new SetShoulderElevatorPosition(constants.k_ShoulderMML1, constants.k_ElevatorMML1)
            .alongWith(new ChangePiplineCommand(2))); //L1
        Button_7.whileTrue(new SetShoulderElevatorPosition(constants.k_ShoulderMML2, constants.k_ElevatorMML2)
            .alongWith(new ChangePiplineCommand(1))); //L2
        Button_8.whileTrue(new SetShoulderElevatorPosition(constants.k_ShoulderMML3, constants.k_ElevatorMML3)
            .alongWith(new ChangePiplineCommand(1))); //L3
        Button_9.whileTrue(new SetShoulderElevatorPosition(constants.k_ShoulderMML4, constants.k_ElevatorMML4)
            .alongWith(new ChangePiplineCommand(1))); //L4

        //intake position is for emergency use only
        Button_10.whileTrue(new SetShoulderElevatorPosition(constants.k_ShoulderMMIntake, constants.k_ElevatorMMIntake)); //Intake Position

        //these buttons set the end effector into climb position and control the climber coming up and down
        /* 
        Button_11.onTrue(new ElevatorSetPositionCommand(constants.k_ElevatorMMClimb)
            .andThen(new ShoulderSetPositionCommand(constants.k_ShoulderMMClimb))).whileTrue(new ClimberUpCommand());
        Button_12.onTrue(new ElevatorSetPositionCommand(constants.k_ElevatorMMClimb)
            .andThen(new ShoulderSetPositionCommand(constants.k_ShoulderMMClimb))).whileTrue(new ClimberDownCommand());
        */

        
        Button_11.onTrue(new ElevatorSetPositionCommand(constants.k_ElevatorMMClimb)
            .andThen(new ShoulderSetPositionCommand(constants.k_ShoulderMMClimb))).whileTrue(new ClimbCommand());
        Button_12.onTrue(new ElevatorSetPositionCommand(constants.k_ElevatorMMClimb)
        .andThen(new ShoulderSetPositionCommand(constants.k_ShoulderMMClimb))).whileTrue(new ClimbPrepCommand());
        

        //these buttons are the emergency intake outtake controls
        Button_13.whileTrue(new IntakeCommand()); //intake
        Button_14.whileTrue(new OuttakeCommand()); //outtake

        Button_15.whileTrue(new SetShoulderElevatorPosition(constants.k_ShoulderMMHomeLow, constants.k_ElevatorMMHomeLow)
            .andThen(new ShoulderElevatorCommand())
            .alongWith(new AlgaeRotateSetPositionCommand(constants.k_AlgaeRotateMMPickup))
            .alongWith(new AlgaeIntakeIntakeCommand())).onFalse(new AlgaeRotateSetPositionCommand(constants.k_AlgaeRotateMMHome));

    //test bindings for the second controller
    
        /* 
        joystick2.a().whileTrue(new WinchManualCommand());
        joystick2.b().whileTrue(new AlgaeIntakeManualCommand());
        joystick2.x().whileTrue(new AlgaeRotateManualCommand());
        */

        /* 
        joystick2.a().whileTrue(new AlgaeRotateSetPositionCommand(constants.k_AlgaeRotateMMHome));//home
        joystick2.b().whileTrue(new AlgaeRotateSetPositionCommand(constants.k_AlgaeRotateMMPickup));//algae pickup pos
        joystick2.x().whileTrue(new AlgaeRotateSetPositionCommand(constants.k_AlgaeRotateMMClimbPrep));//climb prep pos
        joystick2.y().whileTrue(new AlgaeRotateSetPositionCommand(constants.k_AlgaeRotateMMHandoff));//climb handoff pos
        */

        //joystick2.b().whileTrue(new EffectorManualCommand());
        //joystick2.y().whileTrue(new ClimberManualCommand());
        //joystick2.x().whileTrue(new ShoulderManualCommand());
        //joystick2.leftBumper().whileTrue(new ElevatorManualCommand());
        
        //manual set position commands
        /*
        joystick2.leftTrigger().whileTrue(new ElevatorSetPositionCommand(5));
        joystick2.povUp().whileTrue(new ElevatorSetPositionCommand(10));
        joystick2.povRight().whileTrue(new ElevatorSetPositionCommand(30));
        joystick2.povDown().whileTrue(new ElevatorSetPositionCommand(50));
        joystick2.povLeft().whileTrue(new ElevatorSetPositionCommand(80));
        */

        /*
        joystick2.leftTrigger().whileTrue(new ShoulderSetPositionCommand(-.03));
        joystick2.povUp().whileTrue(new ShoulderSetPositionCommand(-0.11));
        joystick2.povRight().whileTrue(new ShoulderSetPositionCommand(0.1));
        joystick2.povDown().whileTrue(new ShoulderSetPositionCommand(0.1));
        joystick2.povLeft().whileTrue(new ShoulderSetPositionCommand(0.15));
        */
    }

    private void namedcommands() {//check if these all works then delete this comment
        NamedCommands.registerCommand("Intake Position", new SetShoulderElevatorPosition(constants.k_ShoulderMMIntakeAuton, constants.k_ElevatorMMIntakeAuton)
        .andThen(new ShoulderElevatorCommand()));
        NamedCommands.registerCommand("Intake", new IntakeCommand());
        NamedCommands.registerCommand("Home", new SetShoulderElevatorPosition().andThen(new ShoulderElevatorCommand()));
        NamedCommands.registerCommand("L4 Stage", new SetShoulderElevatorPosition(constants.k_ShoulderMML4, constants.k_ElevatorMML4)
            .andThen(new ShoulderElevatorCommand()));
        
        //fake L4 Command used for testing
        //NamedCommands.registerCommand("L4 Stage", new SetShoulderElevatorPosition(constants.k_ShoulderMML2, constants.k_ElevatorMML2)
            //.andThen(new ShoulderElevatorCommand()));
        
        NamedCommands.registerCommand("Outtake", new OuttakeCommand());
        NamedCommands.registerCommand("Auto Align", (drivetrain.applyRequest(() -> robotcentricfacingangle
            .withVelocityY(constants.k_y_power * MinSpeed)
            .withVelocityX(constants.k_x_target * .05)
            .withTargetDirection(constants.k_steering_target) //this would be the angle to line up with
            ).ignoringDisable(true)).withTimeout(2.5) // tune me timeout
            .alongWith(new AutoAlignCommand(drivetrain).repeatedly()).withTimeout(2.5));// tune me timeout
        NamedCommands.registerCommand("PreSeed AutoAlign", new AutoAlignCommand(drivetrain));
        NamedCommands.registerCommand("Set LL Pipeline", new ChangePiplineCommand(0));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    //depreciated code that is left in here so we can real quick go back to it if driveteam doesnt like new code
    public double getMinOrMaxSpeed() {
        if (constants.k_ElevatorIsDown) {
            return MaxSpeed;
        } else {
            return MinSpeed;
        }
    }
    
    //this mess uses the basic graphing math to figure out where we should set the max speed to
    //ask matthew if you really want to know
    public double getCalculatedMaxSpeed() {
        //declaring the calculatedMaxSpeed and lowestDivisor
        double calculatedMaxSpeed;
        double lowestDivisor = 1.0;

        //y = mx + b slope formula
        //uses the upper point of (max elevator point, 3), and the lower point of (min elvator point, 1)
        double m = 2/(110 - constants.k_ElevatorMMIntake);
        double b = 3 - (m * 110);
        double y = (m * elevatorsubsystem.getPosition()) + b;
        
        //this makes sure the value never dips below the lowestDivisor value
        if (y < lowestDivisor) {
            calculatedMaxSpeed = MaxSpeed/lowestDivisor;
        } else {
            calculatedMaxSpeed = MaxSpeed/y;
        }
        
        return calculatedMaxSpeed;

        //return MaxSpeed / (((2/(110 - constants.k_ElevatorMMIntake)) * elevatorsubsystem.getPosition()) + .8);
    }

    //getting the alliance that is currently selected in the driver station and returning it as an integer
    private int isAllianceRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Red) {
      return 1;
    } else {
      return 0;
    }
  }
}