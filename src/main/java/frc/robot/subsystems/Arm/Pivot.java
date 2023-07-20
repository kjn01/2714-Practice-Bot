package frc.robot.subsystems.Arm;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Arm.PivotStateMachine;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Pivot extends SubsystemBase {
    public CANSparkMax PivotMotor;
    public AbsoluteEncoder PivotEncoder;
    public ProfiledPIDController PivotController;
    private ArmFeedforward PivotFF;
    private Constraints FarConstraints = new Constraints(12, 9);
    private Constraints CloseConstraints = new Constraints(36, 36);
    private double kLoopTime = 0.020;


    private final LinearSystem<N2, N1, N1> m_plant =
      LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(2), 1.65, 225.0);
      private final KalmanFilter<N2, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N1(),
          m_plant,
          VecBuilder.fill(0.015, 0.17), // How accurate we
          // think our model is, in radians and radians/sec
          VecBuilder.fill(0.01), // How accurate we think our encoder position
          // data is. In this case we very highly trust our encoder position reading.
          0.020);

      // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N2, N1, N1> m_controller =
        new LinearQuadraticRegulator<>(
          m_plant,
          VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), // Q elms.
          // Position and velocity error tolerances, in radians and radians per second. Decrease this
          // to more heavily penalize state excursion, or make the controller behave more
          // aggressively. In this example we weight position much more highly than velocity, but this
          // can be tuned to balance the two.
          VecBuilder.fill(12.0), // R elms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.
    private final LinearSystemLoop<N2, N1, N1> m_loop =
        new LinearSystemLoop<>(m_plant, m_controller, m_observer, 12.0, kLoopTime);
    private final Pivot m_pivot = new Pivot();
    private final PivotStateMachine m_StateMachine = new PivotStateMachine(m_pivot);


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
    public boolean nearGoal() {
        return Math.abs(PivotEncoder.getPosition()- PivotController.getGoal().position) < Units.degreesToRadians(8);
    }
    public void setTargetAngle(double targetAngle){
        Constraints selectedConstraint = (Math.abs(targetAngle - getAngle()) > Units.degreesToRadians(10) ? FarConstraints : CloseConstraints);
        PivotController.setConstraints(selectedConstraint);

        PivotController.setGoal(new State(targetAngle, 0));
    }
    // public void setNextReference(double targetReference){	
    //     m_loop.setNextR(VecBuilder.fill(targetReference));
        // PivotMotor.setVoltage(PivotController.calculate(getAngle(),PivotController.getGoal()) + PivotFF.calculate(PivotController.getSetpoint().position, 0));	
    // }	
    public void setCalculatedVoltage() {
        
        m_loop.setNextR(VecBuilder.fill());
        // pStateMachine.update(...); // Give it what it needs to update (maybe, if it doesn't contain that already)

    }
    
    public InstantCommand setPresetCommand(double armPreset) {
        return new InstantCommand(() -> setPos(armPreset));
    }
    //position change methods
    public Command BackToTransfer(double  backScoreLevelPosition) {
        CommandBase sequence = new SequentialCommandGroup(
          new WaitUntilCommand(() -> m_pivot.nearGoal()).deadlineWith(setPresetCommand(-60))); // backtobackintermediateposition
          //new WaitUntilCommand(() -> shoulder.atSetpoint() && elbow.atSetpoint()));
        sequence.addRequirements(m_pivot);
        return sequence;
    }
    public Command BackToStow() {
        return new SequentialCommandGroup(
          new WaitUntilCommand(() -> m_pivot.nearGoal()).deadlineWith(setPresetCommand(-58)));//stowpos
    }
    public Command TransferToStow() {
        return new SequentialCommandGroup(
          new WaitUntilCommand(() -> m_pivot.nearGoal()).deadlineWith(setPresetCommand(-58)));//stowpos
    }
    public Command StowToTransfer(double transferScoreLevelPosition) {
        return new SequentialCommandGroup(
          new WaitUntilCommand(() -> m_pivot.nearGoal()).deadlineWith(setPresetCommand(transferScoreLevelPosition)));
    }
    

    
      
    @Override	
    public void periodic() {	
        setCalculatedVoltage();	
    }

}
