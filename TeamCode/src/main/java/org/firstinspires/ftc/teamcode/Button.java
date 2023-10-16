package org.firstinspires.ftc.teamcode;

/**
 * The class defining Buttons on a Controller
 */
public class Button {
    private boolean state;
    private boolean previousState;

    /**
     * Whether the button is pressed
     * @return if the button is pressed, false otherwise
     */
    public boolean get() {
        return state;
    }

    /**
     * Whether the button's state just changed
     * @return if the button's state just changed, false otherwise
     */
    public boolean stateJustChanged() {
        return state != previousState;
    }

    /**
     * Whether the button was just pressed
     * @return if the button was just pressed, false otherwise
     */
    public boolean wasJustPressed() {
        return state && !previousState;
    }

    /**
     * Whether the button was just released
     * @return if the button was just released, false otherwise
     */
    public boolean wasJustReleased() {
        return !state && previousState;
    }

    /**
     * Set the button's previous state to its current state
     */
    public void setPreviousState() {
        previousState = state;
    }

    /**
     * Set the button's current state
     * @param state the state to set it to
     */
    public void setCurrentState( boolean state ) {
        this.state = state;
    }

    /**
     * Return the current state, for telemetry
     * @return the value of the button as String
     */
    public String toString()
    {
        return String.valueOf( get() );
    }
}
