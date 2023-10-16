package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GameController {
    private Gamepad gamepad;
    public Button y;
    public Button x;
    public Button b;
    public Button a;
    public Button dpad_up;
    public Button dpad_down;
    public Button dpad_left;
    public Button dpad_right;
    public Button left_bumper;
    public Button right_bumper;
    public Button start;
    public Button back;
    public Button right_stick_button;
    public Button left_stick_button;
    public double left_stick_x;
    public double left_stick_y;
    public double right_stick_x;
    public double right_stick_y;
    public double right_trigger;
    public double left_trigger;

    public GameController(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void updateCurrentValues() {
        y.setCurrentState(gamepad.y);
        x.setCurrentState(gamepad.x);
        b.setCurrentState(gamepad.b);
        a.setCurrentState(gamepad.a);
        dpad_up.setCurrentState(gamepad.dpad_up);
        dpad_down.setCurrentState(gamepad.dpad_down);
        dpad_left.setCurrentState(gamepad.dpad_left);
        dpad_right.setCurrentState(gamepad.dpad_right);
        left_bumper.setCurrentState(gamepad.left_bumper);
        right_bumper.setCurrentState(gamepad.right_bumper);
        start.setCurrentState(gamepad.start);
        back.setCurrentState(gamepad.back);
        right_stick_button.setCurrentState(gamepad.right_stick_button);
        left_stick_button.setCurrentState(gamepad.left_stick_button);
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        right_trigger = gamepad.right_trigger;
        left_trigger = gamepad.left_trigger;
    }

    public void updatePreviousValues() {
        y.setPreviousState();
        x.setPreviousState();
        b.setPreviousState();
        a.setPreviousState();
        dpad_up.setPreviousState();
        dpad_down.setPreviousState();
        dpad_left.setPreviousState();
        dpad_right.setPreviousState();
        left_bumper.setPreviousState();
        right_bumper.setPreviousState();
        start.setPreviousState();
        back.setPreviousState();
        right_stick_button.setPreviousState();
        left_stick_button.setPreviousState();
    }
}
