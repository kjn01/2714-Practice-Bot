package frc.robot.subsystems.Arm;

import javax.print.event.PrintServiceAttributeListener;

public class LiftStateMachine {
    private enum LiftState {
        LOWERED, RAISED, LOWERING, RAISING, IDLE
    }

    private LiftState currentState;

    public LiftStateMachine() {
        this.currentState = LiftState.LOWERED;
    }

    public LiftStateMachine(LiftState startingState) {
        this.currentState = startingState;
    }
    
    public int getOutput() {
        switch (currentState) {
            case LOWERED:
                return 0;
            case LOWERING:
                return -1;
            case RAISED:
                return 0;
            case RAISING:
                return 1;
            case IDLE:
                return 0;
            default:
                return 0;
            
        }
    }

    public void handleStateChange(int encoderValue, boolean upButtonPressed, boolean downButtonPressed) {
        switch(currentState) {
            case IDLE:
                if (upButtonPressed) {
                    currentState = LiftState.RAISING;
                }
                if (downButtonPressed) {
                    currentState = LiftState.LOWERING;
                }
                break;
            case LOWERED:
                if (upButtonPressed) {
                    currentState = LiftState.RAISING;
                }
                break;
            case LOWERING:
                if (encoderValue <= 0) {
                    currentState = LiftState.LOWERED;
                } else if (upButtonPressed) {
                    currentState = LiftState.RAISING;
                } else if (!downButtonPressed) {
                    currentState = LiftState.IDLE;
                }
                break;
            case RAISED:
                if (downButtonPressed) {
                    currentState = LiftState.LOWERING;
                }
                break;
            case RAISING:
                break;
            default:
                break;

        }
    }
}
