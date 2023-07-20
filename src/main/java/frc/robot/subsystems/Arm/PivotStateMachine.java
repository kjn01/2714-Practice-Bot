package frc.robot.subsystems.Arm;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PivotStateMachine {
    private Pivot m_pivot;
    
    /**
     * an enum representing the state of the pivot
     * STOW - During this state...
     * TRANSFER -
     * BACK - 
     */
    public enum PivotState {
        STOW, TRANSFER, BACK
      }
    
    public PivotStateMachine(Pivot m_pivot) {
        this.m_pivot = m_pivot;
    }

    
    private PivotState currentPivotState = PivotState.BACK; //will default to TRANSFER 
    private PivotState targetPivotState = PivotState.BACK; // default to TRANSFER 

    public Command setTargetPivotStateCommand(PivotState targetPivotState) { 
        return new InstantCommand(() -> {
          if (this.targetPivotState != targetPivotState || targetPivotState != PivotState.STOW) {
            currentPivotState = this.targetPivotState;
            this.targetPivotState = targetPivotState;
          }
        });
      }


    
    /**
     * This method calculates the sum of two intergers.
     * It can support positive and netative (signed) integer types
     * but does not support deciamls. The order of the inputs does not
     * change the result
     * 
     * @param a the first integer to be used in calculating the sum
     * @param b the second integer to be used in calculating the sum
     * @return the sum of a and b
     */
    public int add(int a, int b) {
      return a + b;
    }

    /**
     * Depending on the current state, this method will return the output command
     * expected by the current state
     * 
     * @param pivotState 
     * @return
     */
    public Command getPivotCommand() {
        switch (currentPivotState) {
          case BACK: switch (targetPivotState) {
            case BACK: return new InstantCommand();
            case TRANSFER: return m_pivot.BackToTransfer(135);//transfer position elbow
            case STOW: return m_pivot.BackToStow();
          }
          case TRANSFER: switch (targetPivotState) {
            case BACK: return m_pivot.TransferToBack(getBackScoreLevelPosition(pivotScoreLevel, cargoType));
            case TRANSFER: return new InstantCommand();
            case STOW: return m_pivot.TransferToStow();
          }
          case STOW: switch (targetPivotState) {
            case BACK: return m_pivot.StowToBack(getBackScoreLevelPosition(pivotScoreLevel, cargoType));
            case TRANSFER: return m_pivot.StowToTransfer(135);//ktransferpos
            case STOW: return new InstantCommand();
          }
        }
        return new InstantCommand();
      }  

    // pivotStateMachine.getPivotCommand(PivotState.STOW);
}
