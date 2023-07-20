package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ClawConstants;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;


public class Claw {
    private CANSparkMax WristMotor;
    private RelativeEncoder WristEncoder;
    private CANSparkMax ClawMotor;
    private DoubleSolenoid ClawSolenoid;

    public enum WristMode{
        CONE, CUBE
    }
    private static WristMode wristMode = WristMode.CUBE;

    public Claw(){
        ClawMotor = new CANSparkMax(ClawConstants.kClawMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
        WristMotor = new CANSparkMax(ClawConstants.kClawMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);

        ClawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);
        WristMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);
        ClawMotor.setInverted(true);
        WristMotor.setInverted(false);
        ClawMotor.enableVoltageCompensation(ClawConstants.kNominalVoltage);
        ClawMotor.burnFlash();
        WristMotor.burnFlash();

        ClawSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            ClawConstants.kClawSolenoidForwardChannel, 
            ClawConstants.kClawSolenoidReverseChannel);
        
    }

    public void ClawIntake() {
        ClawMotor.set(ClawConstants.kIntakeMotorSpeed * ClawConstants.kNominalVoltage);
    }
    private void ClawOuttake() {
        ClawMotor.set(ClawConstants.kOuttakeMotorSpeed);
    }
    public void SetClawStop() {
        ClawMotor.set(0);
    }
    public Command setWristCommand(WristMode targetArmState) {
        return new InstantCommand(() -> setWristCommand(targetArmState));
      }
    public Command getWristCommand(){
        switch (wristMode){
            case CUBE:
                return new InstantCommand(() -> WristEncoder.setPosition(Units.degreesToRadians(250)));
            case CONE:
                return new InstantCommand(() -> WristEncoder.setPosition(Units.degreesToRadians(205)));
                default:
                return new InstantCommand();
        }
    }
    public Command IntakeCube() {
        return new InstantCommand(() -> { 
            ClawIntake();
            setWristCommand(wristMode.CUBE);
          });
    }
    public Command IntakeCone(){
        return new InstantCommand(() -> { 
            ClawIntake();
            setWristCommand(wristMode.CONE);
          });
    }
    public Command Score() {
        return new InstantCommand(() -> {
          ClawOuttake();
        });
      }
}