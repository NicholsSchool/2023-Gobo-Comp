package org.firstinspires.ftc.teamcode;

public class Button {
    private boolean state;
    private boolean previousState;
    public boolean getState() {
        return state;
    }
    public boolean stateJustChanged() {
        return state != previousState;
    }
    public boolean wasJustPressed() {
        return state && !previousState;
    }
    public boolean wasJustReleased() {
        return !state && previousState;
    }
    public void setPreviousState() {
        previousState = state;
    }
    public void setCurrentState( boolean state ) {
        this.state = state;
    }
}
