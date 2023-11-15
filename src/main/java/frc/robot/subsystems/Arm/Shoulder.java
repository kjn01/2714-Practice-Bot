// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;

public class Shoulder extends SubsystemBase {
  /** Creates a new Shoulder. */
  private CANSparkMax rightShoulderMotor;
  private CANSparkMax leftShoulderMotor;
  private AbsoluteEncoder shoulderEncoder;

  private ProfiledPIDController shoulderController;
  private ArmFeedforward shoulderFF;

  public Shoulder() {
    rightShoulderMotor = new CANSparkMax(9, MotorType.kBrushless);
    leftShoulderMotor = new CANSparkMax(10, MotorType.kBrushless);

    rightShoulderMotor.setIdleMode(IdleMode.kBrake);
    leftShoulderMotor.setIdleMode(IdleMode.kBrake);

    leftShoulderMotor.follow(rightShoulderMotor, true);

    shoulderEncoder = rightShoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);  
    
    shoulderEncoder.setPositionConversionFactor(360 * ShoulderConstants.kShoulderGearRatio);

    shoulderEncoder.setZeroOffset(200 * ShoulderConstants.kShoulderGearRatio);

    shoulderController = new ProfiledPIDController(0.01, 0, 0, new Constraints(2, 1));
    shoulderFF = new ArmFeedforward(0.01, 0, 0);
    
  }

  public double getShoulderAngle() {
    return shoulderEncoder.getPosition() / ShoulderConstants.kShoulderGearRatio;
  }

  public void setShoulderAngle(double targetAngle) {
    shoulderController.setGoal(new State(targetAngle, 0));
  }

  public double getShoulderGoalAngle() {
    return shoulderController.getGoal().position;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    rightShoulderMotor.setVoltage(shoulderController.calculate(getShoulderAngle()) + shoulderFF.calculate(shoulderController.getSetpoint().position, 0));

    SmartDashboard.putNumber("Shoulder Angle", getShoulderAngle());
    SmartDashboard.putNumber("Shoulder Goal Angle", getShoulderGoalAngle());
  }
}
