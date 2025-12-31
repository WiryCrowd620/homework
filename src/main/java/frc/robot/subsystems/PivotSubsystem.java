// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PivotSubsystem extends SubsystemBase {
  private final SparkMax m_motor;
  private final DutyCycleEncoder duty;
  private final RelativeEncoder relative;
  
  
  public PivotSubsystem() {

    m_motor = new SparkMax(Constants.PivotConstants.DEVICE_ID, MotorType.kBrushless);
    duty = new DutyCycleEncoder(Constants.PivotConstants.ENCODER_CHANNEL, Constants.PivotConstants.ENCODER_RANGE, Constants.PivotConstants.ENCODER_OFFSET);
    relative = m_motor.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

  m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  config.closedLoop.pid(Constants.PivotConstants.kP, Constants.PivotConstants.kI, Constants.PivotConstants.kD);

  relative.setPosition(angle());
  }


  public void SetPosition(double angle) {
    m_motor.getClosedLoopController().setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }


  public double angle() {
    return duty.get();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
