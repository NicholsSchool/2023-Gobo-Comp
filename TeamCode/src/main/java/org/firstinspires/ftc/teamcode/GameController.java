package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

/**
 * The Class defining a GameController
 */
public class GameController {
    private Gamepad gamepad;
    Button y;
    Button x;
    Button b;
    Button a;
    Button dpad_up;
    Button dpad_down;
    Button dpad_left;
    Button dpad_right;
    Button left_bumper;
    Button right_bumper;
    Button start;
    Button back;
    Button right_stick_button;
    Button left_stick_button;
    double left_stick_x;
    double left_stick_y;
    double right_stick_x;
    double right_stick_y;
    double right_trigger;
    double left_trigger;

    /**
     * Constucts the GameController with a Gamepad
     * @param gamepad the gamepad to monitor
     */
    public GameController(Gamepad gamepad) {
        this.gamepad = gamepad;
        y = new Button();
        x = new Button();
        b = new Button();
        a = new Button();
        dpad_up = new Button();
        dpad_down = new Button();
        dpad_left = new Button();
        dpad_right = new Button();
        left_bumper = new Button();
        right_bumper = new Button();
        start = new Button();
        back = new Button();
        right_stick_button = new Button();
        left_stick_button = new Button();
    }

    /**
     * The R from the (R, theta) of the Left Joystick
     * @return the power in the range [-1, 1]
     */
    public double leftJSPower()
    {
        return Range.clip( Math.sqrt(left_stick_x * left_stick_x + left_stick_y * left_stick_y), -1.0, 1.0 );
    }

    /**
     * The theta from the (R, theta) of the Left Joystick
     * @param isBlue whether we are on Blue Alliance
     * @return the theta in the range [-180, 180]
     */
    public double leftJSAngle( boolean isBlue )
    {
        if( isBlue )
            return Math.toDegrees( Math.atan2(left_stick_y, left_stick_x) );
        else
            return NumWiz.addAngles( Math.toDegrees( Math.atan2(left_stick_y, left_stick_x) ), 180.0 );
    }

    /**
     * updates the GameController fields, call at the start of each loop()
     */
    public void updateValues() {
        y.updateStates(gamepad.y);
        x.updateStates(gamepad.x);
        b.updateStates(gamepad.b);
        a.updateStates(gamepad.a);
        dpad_up.updateStates(gamepad.dpad_up);
        dpad_down.updateStates(gamepad.dpad_down);
        dpad_left.updateStates(gamepad.dpad_left);
        dpad_right.updateStates(gamepad.dpad_right);
        left_bumper.updateStates(gamepad.left_bumper);
        right_bumper.updateStates(gamepad.right_bumper);
        start.updateStates(gamepad.start);
        back.updateStates(gamepad.back);
        right_stick_button.updateStates(gamepad.right_stick_button);
        left_stick_button.updateStates(gamepad.left_stick_button);
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = -gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = -gamepad.right_stick_y;
        right_trigger = gamepad.right_trigger;
        left_trigger = gamepad.left_trigger;
    }
}
