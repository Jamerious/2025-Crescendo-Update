// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.*;
import com.revrobotics.config.*;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.util.sendable.Sendable;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final SparkMax wrist;

  private final SparkMaxConfig wristConfig;

  private final SparkClosedLoopController wristController;

  private final ClosedLoopConfig wristControllerConfig;

  private final SmartMotionConfig wristControllerSmartConfig;
  
  private DutyCycleEncoder throughBore;

  public Wrist() {

    wrist = new SparkMax(WRIST_ID, SparkLowLevel.MotorType.kBrushless);
   
    wristConfig = new SparkMaxConfig();

    wristControllerConfig = new ClosedLoopConfig();

    wristControllerSmartConfig = new SmartMotionConfig();

    wristConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);

    wristController = wrist.getClosedLoopController();
    wristControllerConfig.p(0.01);
    //wristControllerSmartConfig.allowedClosedLoopError(0.01, null);
   

    throughBore = new DutyCycleEncoder(2);
  }


    public void rotateWrist(double speed){
        wrist.set(speed);
      }
    
    public void rotateWristToPosition(double position){
    wristController.setReference(position, ControlType.kPosition);
  }

  public DutyCycleEncoder getthroughBore(){
    return throughBore;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   @Override
    public void initSendable(SendableBuilder sendableBuilder) {
      sendableBuilder.addDoubleProperty("Wrist pos", ()-> throughBore.get(), null);
      sendableBuilder.addDoubleProperty("Wrist Current", () -> wrist.getOutputCurrent(), null);
    }
}
