// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.EventMarker;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.DriverProfile;
// import frc.robot.commands.ShooterShoot;
// import frc.robot.commands.ShooterStart;
// import frc.robot.commands.ShooterStop;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSlideSubsystem;
// import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MiddleWheelSubsystem;
//  import frc.robot.subsystems.ShooterFlyWheelSubsystem;
 import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SysIDShooter;
// import frc.robot.commands.HopperShooterCommand;
import yams.mechanisms.positional.Elevator;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

   // private final LimelightSubsystem vision = new LimelightSubsystem();


    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
         public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    //      public final HopperSubsystem m_HopperSubsystem = new HopperSubsystem();
    //     public final ShooterFlyWheelSubsystem m_ShooterFlyWheelSubsystem = new ShooterFlyWheelSubsystem();
       public final MiddleWheelSubsystem m_MiddleWheelSubsystem = new MiddleWheelSubsystem();
         public final IntakeSlideSubsystem m_IntakeSlideSubsystem = new IntakeSlideSubsystem();
        //  public final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

         public final SysIDShooter m_SysIDShooter = new SysIDShooter();
        

        //intake slide subsystem bricks the entire robot. you cant deploy anything.

    /* Path follower */
    private final SendableChooser<Command> autoChooser;



    public RobotContainer() {
        SignalLogger.start();
        SignalLogger.setPath("/home/lvuser/logs");
        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        autoChooser = AutoBuilder.buildAutoChooser("box");
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        // NamedCommands.registerCommand("Shooter", m_Shooter2Subsystem.setVelocity(RPM.of(1000))); //here's how you do yams since they have built in commands
        // NamedCommands.registerCommand("stop shooter", m_Shooter2Subsystem.setVelocity(RPM.of(0)));
        // NamedCommands.registerCommand("start hopper", new InstantCommand(() -> m_HopperSubsystem.start())); //heres how u do the reg subsystem
        // NamedCommands.registerCommand("stop hopper", new InstantCommand(() -> m_HopperSubsystem.stop()));
        // NamedCommands.registerCommand("intake out", m_IntakeSubsystem.setHeightAndStop(Inches.of(6)));
        // NamedCommands.registerCommand("intake in", m_IntakeSubsystem.setHeightAndStop(Inches.of(0)));
        // NamedCommands.registerCommand("start middle wheel", new InstantCommand(() -> m_MiddleWheelSubsystem.start()));
        // // NamedCommands.registerCommand("stop middle wheel", new InstantCommand(() -> m_MiddleWheelSubsystem.stop()));
        // NamedCommands.registerCommand("elevator up", m_ElevatorSubsystem.setHeightAndStop(Inches.of(8)));
        // NamedCommands.registerCommand("elevator zero", m_ElevatorSubsystem.setHeightAndStop(Inches.of(0)));

        // NamedCommands.registerCommand("hopperShooter", new HopperShooterCommand(m_HopperSubsystem, m_Shooter2Subsystem)); //heres how you put a regular command


        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
        
    }

    private void configureBindings() {
        


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

            joystick.rightTrigger().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed/10) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed/10) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
            );
        

        // joystick.leftBumper().whileTrue(
        //     //new InstantCommand(() -> System.out.println(-LimelightHelpers.getTX("limelight")) ) 
        //     new InstantCommand(() -> System.out.println(vision.getCenterReefTx("limelight")) )
            
        // );

        // joystick.a().debounce(0.2).whileTrue(
        //     drivetrain.applyRequest(() -> forwardStraight
        //         .withRotationalRate(-vision.getCenterReefTx("limelight")/Constants.VisionProfile.hubProportionalTx)
        //         //.withVelocityX(vision.getHubTA("limelight")*.5) // Reduced speed for fine adjustments
        //         .withVelocityY(joystick.getLeftY())
        //     )
        // );


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // //SIGNAL LOGGER TURN ON!!!!!!!!!!!!!
        // joystick.leftTrigger().onTrue(Commands.runOnce(SignalLogger::start));
        // joystick.rightTrigger().onTrue(Commands.runOnce(SignalLogger::stop));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // joystick.y().whileTrue(m_SysIDShooter.sysIdDynamic(Direction.kForward));
        // joystick.x().whileTrue(m_SysIDShooter.sysIdDynamic(Direction.kReverse));
        // joystick.a().whileTrue(m_SysIDShooter.sysIdQuasistatic(Direction.kForward));
        // joystick.b().whileTrue(m_SysIDShooter.sysIdQuasistatic(Direction.kReverse));

        //testing motor
        // joystick.x().whileTrue(new InstantCommand( () -> m_ShooterSubsystem.start())); 
        // joystick.b().whileTrue(new InstantCommand( () -> m_ShooterSubsystem.stop()));
        // joystick.y().onTrue(new InstantCommand( () -> m_ShooterSubsystem.increasetestingspeed()));
        // joystick.a().onTrue(new InstantCommand( () -> m_ShooterSubsystem.decreasetestingspeed()));
        // joystick.rightBumper().onTrue(new InstantCommand( () -> m_ShooterSubsystem.reverse()));

        
        // Schedule `setVelocity` when the Xbox controller's B button is pressed,
        // cancelling on release.
        // joystick.a().whileTrue(m_Shooter2Subsystem.setVelocity(RPM.of(60)));
        // joystick.b().whileTrue(m_Shooter2Subsystem.setVelocity(RPM.of(1000)));
        // // Schedule `set` when the Xbox controller's B button is pressed,
        // // cancelling on release.
        // joystick.x().whileTrue(m_Shooter2Subsystem.set(0.3));
        // joystick.y().whileTrue(m_Shooter2Subsystem.set(-0.3));
        // joystick.rightBumper().whileTrue(m_Shooter2Subsystem.set(0));

        //intake
        // joystick.x().whileTrue(m_IntakeSubsystem.setHeightAndStop(Inches.of(2)));
        // joystick.b().whileTrue(m_IntakeSubsystem.setHeightAndStop(Inches.of(0)));
        // joystick.rightBumper().onTrue(m_IntakeSubsystem.set(0));

        //elevator
        // joystick.x().onTrue(m_ElevatorSubsystem.setHeightAndStop(Inches.of(33)));
        // joystick.b().onTrue(m_ElevatorSubsystem.setHeightAndStop(Inches.of(0)));
        // joystick.rightBumper().onTrue(m_ElevatorSubsystem.set(0));
        
        //testing the shooter
        // joystick.x().onTrue(new ShooterShoot(m_ShooterFlyWheelSubsystem, m_ShooterSubsystem));
        // joystick.b().onTrue(new ShooterStop(m_ShooterFlyWheelSubsystem, m_ShooterSubsystem));
        // joystick.x().onTrue((new InstantCommand( () -> m_ShooterSubsystem.reverse())));
        // // //"reverse" is it moving in the correct direction
        // joystick.b().onTrue((new InstantCommand( () -> m_ShooterSubsystem.stop())));
        // joystick.a().onTrue((new InstantCommand( () -> m_ShooterSubsystem.start())));
        //middle wheel ?
    //    joystick.leftTrigger().whileTrue(new InstantCommand( () -> m_MiddleWheelSubsystem.start())); 
    //     joystick.leftTrigger().onFalse(new InstantCommand( () -> m_MiddleWheelSubsystem.stop()));
    //     joystick.y().onTrue(new InstantCommand( () -> m_MiddleWheelSubsystem.increasetestingspeed()));
    //     joystick.a().onTrue(new InstantCommand( () -> m_MiddleWheelSubsystem.decreasetestingspeed()));
    //     joystick.rightBumper().onTrue(new InstantCommand( () -> m_MiddleWheelSubsystem.reverse()));

        // Reset the field-centric heading on left bumper press.
        joystick.rightBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}