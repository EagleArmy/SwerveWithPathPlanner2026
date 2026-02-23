// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX ShooterMotor2 = new TalonFX(ShooterConstants.ShooterMotorID2);
    private final TalonFX ShooterIntakeMotor = new TalonFX(ShooterConstants.ShooterIntakeID);
    private final TalonFX ShooterFlywheelMotor = new TalonFX(ShooterConstants.ShooterFlywheelMotorID);

        public double testingspeed;
        public double intakeSpeed;
    public ShooterSubsystem() 

    
    {
        testingspeed = 1;
        intakeSpeed = .5;
       addChild("ShooterMotor2", ShooterMotor2);

       var ShooterMotor2Configuration = new TalonFXConfiguration();
       ShooterMotor2Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       ShooterMotor2Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       ShooterMotor2.getConfigurator().apply( ShooterMotor2Configuration );
       
        var ShooterMotorIntakeConfiguration = new TalonFXConfiguration();
       ShooterMotorIntakeConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       ShooterMotorIntakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       ShooterIntakeMotor.getConfigurator().apply( ShooterMotorIntakeConfiguration );

               var ShooterMotorFlywheelConfiguration = new TalonFXConfiguration();
       ShooterMotorFlywheelConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       ShooterMotorFlywheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       ShooterFlywheelMotor.getConfigurator().apply( ShooterMotorFlywheelConfiguration );

    var slot0ConfigsRight = ShooterMotor2Configuration.Slot0;
    slot0ConfigsRight.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsRight.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsRight.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsRight.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
    slot0ConfigsRight.kI = 0; // no output for integrated error
    slot0ConfigsRight.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    var slot0ConfigsIntake = ShooterMotorIntakeConfiguration.Slot0;
    slot0ConfigsIntake.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsIntake.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsIntake.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsIntake.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
    slot0ConfigsIntake.kI = 0; // no output for integrated error
    slot0ConfigsIntake.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    var slot0ConfigsFlywheel = ShooterMotorFlywheelConfiguration.Slot0;
    slot0ConfigsFlywheel.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsFlywheel.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsFlywheel.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsFlywheel.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
    slot0ConfigsFlywheel.kI = 0; // no output for integrated error
    slot0ConfigsFlywheel.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output


    // // set Motion Magic settings
    var motionMagicConfigsRight = ShooterMotor2Configuration.MotionMagic;
    motionMagicConfigsRight.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigsRight.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsRight.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // // set Motion Magic settings
    var motionMagicConfigsIntake = ShooterMotorIntakeConfiguration.MotionMagic;
    motionMagicConfigsIntake.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigsIntake.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsIntake.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)

      // // set Motion Magic settings
    var motionMagicConfigsFlywheel = ShooterMotorFlywheelConfiguration.MotionMagic;
    motionMagicConfigsIntake.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigsIntake.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsIntake.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // 20, 40, 400
    // 8000, 16000, 160000
    ShooterMotor2.getConfigurator().apply(ShooterMotor2Configuration);
    ShooterIntakeMotor.getConfigurator().apply(ShooterMotorIntakeConfiguration);
    ShooterFlywheelMotor.getConfigurator().apply(ShooterMotorFlywheelConfiguration);

    }

    public void start()
    {
        ShooterFlywheelMotor.set(-testingspeed);
        ShooterIntakeMotor.set(-testingspeed);
        ShooterMotor2.set(testingspeed); //10 needs to be reversed
        System.out.println("Testing Speed: " + testingspeed);
    }

    public void reverse() 
    {
        ShooterFlywheelMotor.set(testingspeed);
        ShooterIntakeMotor.set(testingspeed);
        ShooterMotor2.set(-testingspeed);
    }

    public void stop()
    {
        ShooterIntakeMotor.set(0);
        ShooterFlywheelMotor.set(0);
        ShooterMotor2.set( 0 );
    }
    public void increasetestingspeed() 
    {
        testingspeed += 0.05;
        System.out.println("Testing Speed: " + testingspeed);
    }
    public void decreasetestingspeed() 
    {
        testingspeed -= 0.05;
        System.out.println("Testing Speed: " + testingspeed);
    }

    @Override
    public void periodic() 
    {
      //SmartDashboard.putNumber("Extender Position", getEncoderPosition());
      //log();
    }

}
