// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.management.relation.Relation;

import com.revrobotics.spark.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

import static frc.robot.Constants.OuttakeConstants;


public class Outtake extends SubsystemBase {
  
  private SparkMax leftMotor;
  private SparkMax rightMotor;

  private SparkMaxConfig leftMotorConfig;
  private SparkMaxConfig rightMotorConfig;

  private RelativeEncoder encoder;

  private SparkClosedLoopController pidController;
    
  private ClosedLoopConfig pidControllerConfig;

  private SmartMotionConfig pidControllerSmartConfig;

  private double velocity;

  /** Creates a new Shooter. */
  public Outtake() {


    leftMotor = new SparkMax(OuttakeConstants.MOTOR_ONE_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(OuttakeConstants.MOTOR_TWO_ID, MotorType.kBrushless);

    leftMotorConfig = new SparkMaxConfig();
    rightMotorConfig = new SparkMaxConfig();

    leftMotorConfig.smartCurrentLimit(OuttakeConstants.CURRENT_LIMIT).inverted(true).idleMode(IdleMode.kCoast);
    rightMotorConfig.smartCurrentLimit(OuttakeConstants.CURRENT_LIMIT).inverted(true).idleMode(IdleMode.kCoast).follow(rightMotor, true);


    encoder = leftMotor.getEncoder();    

    pidController = leftMotor.getClosedLoopController();

    pidControllerConfig = new ClosedLoopConfig();   
    
    pidControllerSmartConfig = new SmartMotionConfig();

    //This didn't help
    pidControllerConfig.p(OuttakeConstants.P_VALUE);
    pidControllerConfig.velocityFF(OuttakeConstants.FF_VALUE);
    pidControllerSmartConfig.allowedClosedLoopError(100, ClosedLoopSlot.kSlot0);
    pidControllerConfig.outputRange(0, 5000);
  }

  public void shootPercent(double speed){
    leftMotor.set(speed);
    // System.out.println("Shooting at " + speed);
  }

  public void stop(){
    leftMotor.set(0);
  }

  public void shootVelocity(double velocity){
    pidController.setReference(velocity, ControlType.kVelocity);
  }

  public boolean isUpToSpeed(double targetSpeed){
    return encoder.getVelocity() >= targetSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Shooter RPM", encoder.getVelocity());
  }
}
