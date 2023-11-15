// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;

public class Shoulder extends SubsystemBase {
  /** Creates a new Shoulder. */
  private CANSparkMax rightShoulderMotor;
  private CANSparkMax leftShoulderMotor;
  private AbsoluteEncoder shoulderEncoder;

  public Shoulder() {
    rightShoulderMotor = new CANSparkMax(10, MotorType.kBrushless);
    leftShoulderMotor = new CANSparkMax(9, MotorType.kBrushless);

    rightShoulderMotor.setIdleMode(IdleMode.kCoast);
    leftShoulderMotor.setIdleMode(IdleMode.kCoast);

    leftShoulderMotor.follow(rightShoulderMotor, true);

    shoulderEncoder = rightShoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);  
    
    shoulderEncoder.setPositionConversionFactor(360 * ShoulderConstants.kShoulderGearRatio);

    shoulderEncoder.setZeroOffset(200 * ShoulderConstants.kShoulderGearRatio);
    
  }

  public double getShoulderAngle() {
    return shoulderEncoder.getPosition() / ShoulderConstants.kShoulderGearRatio;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder Angle", getShoulderAngle());
  }
}
