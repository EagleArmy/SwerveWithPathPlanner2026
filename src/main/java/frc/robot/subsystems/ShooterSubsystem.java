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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX ShooterMotor = new TalonFX(ShooterConstants.ShooterMotorID);
    private final TalonFX ShooterMotor2 = new TalonFX(ShooterConstants.ShooterMotorID2);

        public double testingspeed;
    public ShooterSubsystem() 
    {
        testingspeed = 0.20;
       addChild("ShooterMotor", ShooterMotor);
       addChild("ShooterMotor2", ShooterMotor2);

       var ShooterMotorConfiguration = new TalonFXConfiguration();
       ShooterMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       ShooterMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       ShooterMotor.getConfigurator().apply( ShooterMotorConfiguration );

       var ShooterMotor2Configuration = new TalonFXConfiguration();
       ShooterMotor2Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       ShooterMotor2Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       ShooterMotor.getConfigurator().apply( ShooterMotor2Configuration );

      var slot0ConfigsLeft = ShooterMotorConfiguration.Slot0;
    slot0ConfigsLeft.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsLeft.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsLeft.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsLeft.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
    slot0ConfigsLeft.kI = 0; // no output for integrated error
    slot0ConfigsLeft.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    var slot0ConfigsRight = ShooterMotor2Configuration.Slot0;
    slot0ConfigsRight.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsRight.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsRight.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsRight.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
    slot0ConfigsRight.kI = 0; // no output for integrated error
    slot0ConfigsRight.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    //  set Motion Magic settings
    var motionMagicConfigsLeft = ShooterMotorConfiguration.MotionMagic;
    motionMagicConfigsLeft.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigsLeft.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsLeft.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)
    // 20, 40, 400
    // 8000, 16000, 160000

    // // set Motion Magic settings
    var motionMagicConfigsRight = ShooterMotor2Configuration.MotionMagic;
    motionMagicConfigsRight.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigsRight.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsRight.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)
    // 20, 40, 400
    // 8000, 16000, 160000

    ShooterMotor.getConfigurator().apply(ShooterMotorConfiguration);

    ShooterMotor2.getConfigurator().apply(ShooterMotor2Configuration);

    }

    public void start()
    {
        ShooterMotor.set(-testingspeed);
        ShooterMotor2.set(testingspeed); //10 needs to be reversed
        System.out.println("Testing Speed: " + testingspeed);
    }

    public void reverse() 
    {
        ShooterMotor.set(testingspeed);
        ShooterMotor2.set(-testingspeed);
    }

    public void stop()
    {
        ShooterMotor.set( 0 );
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
