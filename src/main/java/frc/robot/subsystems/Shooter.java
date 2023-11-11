// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private CANSparkMax pivotMotor;
  private CANSparkMax topFlywheelMotor;
  private CANSparkMax bottomFlywheelMotor;
  private CANSparkMax kickerMotor;

  private AbsoluteEncoder pivotEncoder;

  private PIDController pivotController;

  double target;

  public Shooter() {

    pivotMotor = new CANSparkMax(15, MotorType.kBrushless);
    topFlywheelMotor = new CANSparkMax(16, MotorType.kBrushless);
    bottomFlywheelMotor = new CANSparkMax(17, MotorType.kBrushless);
    kickerMotor = new CANSparkMax(13, MotorType.kBrushless);

    pivotMotor.setIdleMode(IdleMode.kBrake);
    topFlywheelMotor.setIdleMode(IdleMode.kCoast);
    bottomFlywheelMotor.setIdleMode(IdleMode.kCoast);
    kickerMotor.setIdleMode(IdleMode.kCoast);

    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

    pivotEncoder.setPositionConversionFactor(360 * ShooterConstants.kPivotGearRatio);

    pivotEncoder.setZeroOffset(160 * ShooterConstants.kPivotGearRatio); // TBD

    pivotController = new PIDController(0.055, 0, 0);

    target = 0.0;

  }

  public double getPivotAngle() {
    return (pivotEncoder.getPosition() - ShooterConstants.kPivotOffset * ShooterConstants.kPivotGearRatio) / ShooterConstants.kPivotGearRatio;
  }

  public void setPivotAngle(double target) {
    this.target = target;
    pivotController.setSetpoint(target);
  }

  public boolean nearSetpoint() {
    return Math.abs(getPivotAngle() - target) < 5;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pivotMotor.setVoltage(pivotController.calculate(getPivotAngle()));
    SmartDashboard.putNumber("Pivot Position", getPivotAngle());
  }
}
