// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */

  private DriveSubsystem m_drive;
  private Limelight m_limelight;

  ProfiledPIDController YController;
  ProfiledPIDController XController;
  ProfiledPIDController ThetaController;

  public AutoAlign(DriveSubsystem m_drive, Limelight m_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.m_limelight = m_limelight;

    YController = new ProfiledPIDController(0.01, 0, 0, new Constraints(0.1, 0.1));
    XController = new ProfiledPIDController(0.01, 0, 0, new Constraints(0.1, 0.1));
    ThetaController = new ProfiledPIDController(0.01, 0, 0, new Constraints(0.1, 0.1));

    addRequirements(m_drive);

    YController.setGoal(0);
    XController.setGoal(1);
    ThetaController.setGoal(0);
    ThetaController.enableContinuousInput(-180, 180);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setAprilTagPipeline();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.isTargetVisible()) {
      m_drive.drive(
        XController.calculate(
          m_limelight.getDistanceToGoalMeters()),
          m_limelight.getYAngleOffsetDegrees(),
          m_drive.getHeading(),
          true,
          false);
          
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return XController.atGoal() && YController.atGoal() && ThetaController.atGoal();
  }
}
