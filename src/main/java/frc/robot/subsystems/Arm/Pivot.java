package frc.robot.subsystems.Arm;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class Pivot extends SubsystemBase{
    public CANSparkMax PivotMotor;
    public AbsoluteEncoder PivotEncoder;
    public ProfiledPIDController PivotController;
    private ArmFeedforward PivotFF;
    private Constraints FarConstraints = new Constraints(12, 9);
    private Constraints CloseConstraints = new Constraints(36, 36);
    public Pivot(){
        PivotMotor = new CANSparkMax(PivotConstants.kPivotMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
        PivotMotor.setInverted(false);
        PivotMotor.setSmartCurrentLimit(PivotConstants.kPivotMotorCurrentLimit);
        PivotEncoder.setPositionConversionFactor(PivotConstants.kPivotPositionConversionFactor);
        
        PivotEncoder = PivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        PivotMotor.setIdleMode(IdleMode.kBrake);
        PivotController = new ProfiledPIDController(0.001, 0,  0, new TrapezoidProfile.Constraints(0.1, 0.1));
        PivotFF = new ArmFeedforward(0,0.35,4.38,0.03);
        PivotEncoder.setZeroOffset(PivotConstants.kPivotEncoderZeroOffset);
    }
    public double convertTicksToAngle(double angle){
        double newAngle = angle;
        newAngle -= PivotConstants.kPivotEncoderZeroOffset;
        return newAngle / PivotConstants.kPivotGearRatio;
    }
    public double getAngle(){
        return convertTicksToAngle(PivotEncoder.getPosition());
    }
    public void setPos(double goal){	
        if(PivotController.getP() == 0) {PivotController.setP(5);}	
        PivotController.setGoal(goal);	
    }
    public boolean atGoal() {	
        return PivotController.atGoal();	
    }
    public void setTargetAngle(double targetAngle){
        Constraints selectedConstraint = (Math.abs(targetAngle - getAngle()) > Units.degreesToRadians(10) ? FarConstraints : CloseConstraints);
        PivotController.setConstraints(selectedConstraint);

        PivotController.setGoal(new State(targetAngle, 0));
    }
    public void setCalculatedVoltage(){	
        PivotMotor.setVoltage(PivotController.calculate(getAngle(),PivotController.getGoal()) + PivotFF.calculate(PivotController.getSetpoint().position, 0));	
    }	
    @Override	
    public void periodic() {	
    setCalculatedVoltage();	
    }
}
