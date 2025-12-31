// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  

  private final SparkMax m_motor;
  private final DutyCycleEncoder duty;
  private final RelativeEncoder relative;
  private final ElevatorFeedforward eFeedForward;


  public ElevatorSubsystem() {
    m_motor = new SparkMax(Constants.ElevatorConstants.DEVICE_ID, MotorType.kBrushless);
    relative = m_motor.getEncoder();
    duty = new DutyCycleEncoder(Constants.ElevatorConstants.ENCODER_CHANNEL, Constants.ElevatorConstants.ENCODER_RANGE, Constants.ElevatorConstants.ENCODER_OFFSET);
    eFeedForward = new ElevatorFeedforward(Constants.ElevatorConstants.kS, 
      Constants.ElevatorConstants.kG, 
      Constants.ElevatorConstants.kV, 
      Constants.ElevatorConstants.kA);

    SparkMaxConfig config = new SparkMaxConfig();

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.closedLoop.pidf(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD, Constants.ElevatorConstants.FF);

    relative.setPosition(duty.get());


  }

  public void SetPosition(double angle) {
    m_motor.getClosedLoopController().setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }

  public double ffCalc() {
    double current = relative.getPosition() * 360;
    double targetVelocityDegPerSec = relative.getVelocity() / 60;
    return eFeedForward.calculate( 
      Math.toRadians(current),  
      Math.toRadians(targetVelocityDegPerSec) 

  ); 
  }


  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  // hello

}
