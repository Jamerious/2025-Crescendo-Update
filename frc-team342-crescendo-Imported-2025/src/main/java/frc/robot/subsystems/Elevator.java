// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.util.sendable.SendableBuilder;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class Elevator extends SubsystemBase {

  private final SparkMax elevator_left;
  private final SparkMax elevator_right;

  private final SparkMaxConfig elevator_left_Config;
  private final SparkMaxConfig elevator_right_Config;


  private final SparkClosedLoopController pid_elevator;

  private boolean climbMode = false;

  private final RelativeEncoder encoder;
  
  /** Creates a new Elevator. */
  public Elevator() {

    elevator_left = new SparkMax(LEFT_ELEV_ID, MotorType.kBrushless);
    elevator_right = new SparkMax(RIGHT_ElEV_ID, MotorType.kBrushless);


    //elevator_left.restoreFactoryDefaults();
    pid_elevator = elevator_left.getClosedLoopController();

    elevator_left_Config = new SparkMaxConfig();
    elevator_right_Config = new SparkMaxConfig();

    elevator_left_Config.idleMode(IdleMode.kBrake).smartCurrentLimit(30).inverted(true);
    elevator_right_Config.idleMode(IdleMode.kBrake).smartCurrentLimit(30).inverted(true).follow(elevator_left, true);

    encoder = elevator_left.getEncoder();
  }

  public Boolean getClimbMode(){
    return climbMode;
  }

  public Command toggleClimbMode(){
    return runEnd( () -> {}, ()-> {climbMode = !climbMode;});
  }

   public void raiseElevatorwithSpeed(double speed){
    elevator_left.set(speed);
  }

  public void raiseElevatorToPosition(double pos){
    pid_elevator.setReference(pos, ControlType.kPosition);
  }

  public void holdPosition (){
    pid_elevator.setReference(elevator_left.getEncoder().getPosition(), ControlType.kPosition);
  }


  public double getElevatorEncoder(){
    return elevator_left.getEncoder().getPosition();
  }


  @Override
  public void periodic() {}

  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.addBooleanProperty("Climb Mode", () -> climbMode, null);
  }
}
