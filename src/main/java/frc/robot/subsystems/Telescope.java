package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.TelescopeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase{
    public CANSparkMax telescopeMotor;
    public RelativeEncoder telescopeEncoder;
    public SimpleMotorFeedforward telescopeFF = new SimpleMotorFeedforward(0,0,0);//to be tuned
    //tbd vvv
    public ProfiledPIDController telescopeController = 
        new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0.1, 0.1));

    public Telescope(){
        telescopeEncoder = telescopeMotor.getEncoder();
        telescopeMotor = new CANSparkMax(TelescopeConstants.kTelescopeMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless); 
        telescopeEncoder.setInverted(false);
        telescopeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        telescopeMotor.burnFlash();
        telescopeController.disableContinuousInput();
    }
    public double getPosition(){
        return telescopeEncoder.getPosition();
    }
    public void setTargetPosition(double target){
        
        telescopeController.setGoal(new State(target,0));
    }
    public boolean atGoal() {
        return telescopeController.atGoal();
      }
    private void setCalculatedVoltage() {
        double voltage = 
          telescopeController.calculate(getPosition())
          + telescopeFF.calculate(telescopeController.getSetpoint().position, 0);
    
        telescopeMotor.setVoltage(voltage);
      }  
    @Override
    public void periodic() {
        setCalculatedVoltage();
      }
}
