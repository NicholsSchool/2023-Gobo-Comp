package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop: Robot Genesis")
public class GenesisTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private GameController driverOI;
    private GameController operatorOI;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        driverOI = new GameController(gamepad1);
        operatorOI = new GameController(gamepad2);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        driverOI.updateValues();
        operatorOI.updateValues();

        double power = driverOI.leftJSPower();
        double angle = driverOI.leftJSAngle( driverOI.a.get() );
        double turn = driverOI.right_stick_x;

        // Show Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Power", power );
        telemetry.addData("angle", angle);
        telemetry.addData("turn power", turn);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
