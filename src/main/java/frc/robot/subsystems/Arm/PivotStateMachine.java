package frc.robot.subsystems.Pivot;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PivotStateMachine {
    private Pivot m_pivot;
    
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

    
    public Command getPivotCommand(PivotState pivotState) {
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
}
