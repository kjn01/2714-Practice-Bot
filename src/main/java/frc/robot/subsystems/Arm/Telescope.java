// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopeConstants;

public class Telescope extends SubsystemBase {
  /** Creates a new Arm. */

  private CANSparkMax telescopingMotor;
  private RelativeEncoder telescopingEncoder;
  
  private ProfiledPIDController telescopingController = new ProfiledPIDController(0, 0, 0, TelescopeConstants.kTelescopingConstraints); // tune later
  private SimpleMotorFeedforward telescopeFF = new SimpleMotorFeedforward(0, 0, 0); // tune later

  public Telescope() {
    
    telescopingMotor = new CANSparkMax(TelescopeConstants.kTelescopingMotorCanId, MotorType.kBrushless);
    telescopingEncoder = telescopingMotor.getEncoder();

    telescopingMotor.setSmartCurrentLimit(TelescopeConstants.kCurrentLimit);
    telescopingMotor.setIdleMode(IdleMode.kBrake);

    telescopingMotor.burnFlash();

    telescopingController.disableContinuousInput();

  }

  public void setTelescopingPosition(double target) {
    
    telescopingController.setGoal(new State(target, 0));
  }

  public void setCalculatedVoltage() {
    double voltage = telescopingController.calculate(telescopingEncoder.getPosition())
    + telescopeFF.calculate(telescopingEncoder.getVelocity());
    telescopingMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setCalculatedVoltage();

    SmartDashboard.putNumber("Telescope position", telescopingEncoder.getPosition());
  }
}
