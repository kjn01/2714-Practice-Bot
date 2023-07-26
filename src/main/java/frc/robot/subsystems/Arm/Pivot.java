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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Arm.PivotStateMachine;
import frc.robot.subsystems.Arm.PivotStateMachine.PivotState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Pivot extends SubsystemBase {
    public CANSparkMax PivotMotor;
    public AbsoluteEncoder PivotEncoder;
    private double kLoopTime = 0.020;
    private PivotState currentPivotState = PivotState.BACK; // will default to TRANSFER
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
            1.0, // rad/s
            1.0); // rad/s^2
    private final LinearSystem<N2, N1, N1> m_plant = LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(2),
            1.65, 225.0);
    private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            m_plant,
            VecBuilder.fill(0.015, 0.17), // How accurate we
            // think our model is, in radians and radians/sec
            VecBuilder.fill(0.01), // How accurate we think our encoder position
            // data is. In this case we very highly trust our encoder position reading.
            0.020);

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
            m_plant,
            VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), // Q elms.
            // Position and velocity error tolerances, in radians and radians per second.
            // Decrease this
            // to more heavily penalize state excursion, or make the controller behave more
            // aggressively. In this example we weight position much more highly than
            // velocity, but this
            // can be tuned to balance the two.
            VecBuilder.fill(12.0), // R elms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12
            // is a good
            // starting point because that is the (approximate) maximum voltage of a
            // battery.
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
    // lower if using notifiers.
    private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(m_plant, m_controller, m_observer, 12.0,
            kLoopTime);

    private final Pivot m_pivot = new Pivot();
    private final PivotStateMachine m_StateMachine = new PivotStateMachine(m_pivot);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

    public Pivot() {
        PivotMotor = new CANSparkMax(PivotConstants.kPivotMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
        PivotMotor.setInverted(false);
        PivotMotor.setSmartCurrentLimit(PivotConstants.kPivotMotorCurrentLimit);
        PivotEncoder.setPositionConversionFactor(PivotConstants.kPivotPositionConversionFactor);

        PivotEncoder = PivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        PivotMotor.setIdleMode(IdleMode.kBrake);
        PivotEncoder.setZeroOffset(PivotConstants.kPivotEncoderZeroOffset);

    }

    public double convertTicksToAngle(double angle) {
        double newAngle = angle;
        newAngle -= PivotConstants.kPivotEncoderZeroOffset;
        return newAngle / PivotConstants.kPivotGearRatio;
    }

    public double getAngle() {
        return convertTicksToAngle(PivotEncoder.getPosition());
    }

    public void setPos(double goal) {
        m_loop.setNextR(goal);
    }

    public boolean nearGoal() {
        return Math.abs(PivotEncoder.getPosition() - goal.position) < Units.degreesToRadians(8);
    }

    // public void setNextReference(double targetReference){
    // m_loop.setNextR(VecBuilder.fill(targetReference));
    // PivotMotor.setVoltage(PivotController.calculate(getAngle(),PivotController.getGoal())
    // + PivotFF.calculate(PivotController.getSetpoint().position, 0));
    // }
    /**
     * @param
     * NO        WORKING GOAL SETTING RIGHT NOW
     *           below method referenced from
     *           https://github.com/ZachOrr/allwpilib/blob/43d40c6e9e95ed0d7bb5f1baa932af37d4449189/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/statespacearm/Robot.java#L55
     */
    public void setCalculatedVoltage() {
        if (currentPivotState == PivotState.BACK) {
            goal = new TrapezoidProfile.State(135, 0.0);// transfer
        } else {
            if (currentPivotState == PivotState.TRANSFER) {
                if (m_driverController.getAButtonPressed()) {
                    goal = new TrapezoidProfile.State(-30, 0.0); // b
                } else {
                    if (m_driverController.getBButtonPressed()) {
                        goal = new TrapezoidProfile.State(-86, 0.0); // backintake
                    }
                }
            } else {
                if (currentPivotState == PivotState.STOW) {
                    goal = new TrapezoidProfile.State(135, 0.0);
                }
            }
        }

        m_lastProfiledReference = (new TrapezoidProfile(m_constraints, goal, m_lastProfiledReference))
                .calculate(kLoopTime);
        m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

        m_loop.correct(VecBuilder.fill(PivotEncoder.getPosition()));
        double nextVoltage = m_loop.getU(0);
        m_loop.predict(kLoopTime);
        // pStateMachine.update(...); // Give it what it needs to update (maybe, if it
        // doesn't contain that already)

    }

    public InstantCommand setPresetCommand(double armPreset) {
        return new InstantCommand(() -> setPos(armPreset));
    }

    // position change methods
    public Command BackToTransfer(double backScoreLevelPosition) {
        currentPivotState = PivotState.TRANSFER;
        CommandBase sequence = new SequentialCommandGroup(
                new WaitUntilCommand(() -> m_pivot.nearGoal()).deadlineWith(setPresetCommand(-60))); // backtobackintermediateposition
        // new WaitUntilCommand(() -> shoulder.atSetpoint() && elbow.atSetpoint()));
        sequence.addRequirements(m_pivot);
        return sequence;
    }

    public Command BackToStow() {
        currentPivotState = PivotState.STOW;
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> m_pivot.nearGoal()).deadlineWith(setPresetCommand(-58)));// stowpos
        // deadlinewith interrupts the second command when the first command finishes
    }

    public Command TransferToStow() {
        currentPivotState = PivotState.STOW;
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> m_pivot.nearGoal()).deadlineWith(setPresetCommand(-58)));// stowpos
    }

    public Command StowToTransfer(double transferScoreLevelPosition) {
        currentPivotState = PivotState.TRANSFER;
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> m_pivot.nearGoal())
                        .deadlineWith(setPresetCommand(transferScoreLevelPosition)));
    }

    public Command TransferToBack() {
        currentPivotState = PivotState.BACK;
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> m_pivot.nearGoal()).deadlineWith(setPresetCommand(135)));// transfer
    }

    public Command StowToBack() {
        currentPivotState = PivotState.BACK;
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> m_pivot.nearGoal()).deadlineWith(setPresetCommand(-86)));// backintake
    }

    @Override
    public void periodic() {
        setCalculatedVoltage();
    }

}
