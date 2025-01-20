// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.config.BaseConfig;
//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.SparkLowLevel;
//import com.revrobotics.spark.SparkAnalogSensor;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {

  private final SparkMax intake;

  private final SparkMaxConfig intakeConfig;
  
  private DigitalInput intakeSensor;
  private DutyCycleEncoder throughBore;
  private RelativeEncoder encoder;

  private SparkClosedLoopController controller;

  private ClosedLoopConfig controllerConfig;

  private double velocity;
  
  /** Creates a new Intake. */
  public Intake(){
    intake = new SparkMax(INTAKE_MOTOR, SparkLowLevel.MotorType.kBrushless);

    intakeConfig = new SparkMaxConfig();

    controllerConfig = new ClosedLoopConfig();

    intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);

    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

    //intake.setSmartCurrentLimit(60);

    intakeSensor = new DigitalInput(5);
    encoder = intake.getEncoder();

    controller = intake.getClosedLoopController();

    intakeConfig.closedLoop.p(0.01);
    //controllerConfig.p(0.01);
    intakeConfig.closedLoop.p(0.02);
    //controllerConfig.velocityFF(0.02);

    velocity = 0.50;
  }


  //command version
  public Command spinIntake(){
    return runEnd( () -> {
      if(intakeSensor.get()){
        intake.set(-0.7);
      }
      else {
        intake.set(0);
      }
    }, 
    
    () -> {intake.set(0);});
  }

  public Command outtake() {
    return runEnd(() -> {
      intake.set(velocity);
    }, () -> {intake.set(0);});
  }

  public Command softOuttake() {
    return runEnd(() -> {
      intake.set(0.05);
    }, () -> {intake.set(0);});
  }

  public void feedShooter(){
    intake.set(FEED_SHOOTER_SPEED);
  }

  public void hold(){
      intake.set(-0.4);   
  }


  public void stop(){
    intake.set(0);
  }

  /*
   * Returns the value from the intake Sensor
   */
  public boolean getIntakeSensor(){
    return intakeSensor.get();
  }

 

  public boolean isStuck() {
    return intake.getOutputCurrent() > IntakeConstants.DEFAULT_CURRENT; 
  }

  @Override
  public void periodic() {
    

  }

  @Override
    public void initSendable(SendableBuilder sendableBuilder) {
     sendableBuilder.addBooleanProperty("Note Detected", () -> !getIntakeSensor(), null);
     sendableBuilder.addDoubleProperty("Intake RPM", () -> intake.getOutputCurrent(), null);
    }
}
