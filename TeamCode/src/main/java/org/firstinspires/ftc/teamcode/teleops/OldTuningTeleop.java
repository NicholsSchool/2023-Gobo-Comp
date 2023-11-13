package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.GameController;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Constants;

@TeleOp(name="Teleop: Old Tuning")
public class OldTuningTeleop extends OpMode implements Constants
{
    // Declare OpMode members.
    private ElapsedTime runtime;
    private Drivetrain drivetrain;
    private GameController driverOI;
    private GameController operatorOI;
    private String motor;
    private double increment;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize Fields
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap, BLUE_ALLIANCE, 0.0, 0.0);
        driverOI = new GameController(gamepad1);
        operatorOI = new GameController(gamepad2);
        motor = "frontRight";
        increment = 0.1;

        // Initialize Runtime
        runtime = new ElapsedTime();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

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
        drivetrain.updatePose();

        if( driverOI.dpad_up.wasJustPressed() )
            motor = "frontRight";
        else if( driverOI.dpad_right.wasJustPressed() )
            motor = "backRight";
        else if( driverOI.dpad_down.wasJustPressed() )
            motor = "backLeft";
        else if( driverOI.dpad_left.wasJustPressed() )
            motor = "frontLeft";
        else if( driverOI.y.wasJustPressed() )
            increment *= 10;
        else if( driverOI.a.wasJustPressed() )
            increment /= 10;
        else if( driverOI.b.wasJustPressed() )
        {
            switch(motor) {
                case "backRight":
                    drivetrain.backRight.setVelocityPIDFCoefficients( 0, 0, 0,
                            drivetrain.backRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f + increment);
                    break;
                case "backLeft":
                    drivetrain.backLeft.setVelocityPIDFCoefficients( 0, 0, 0,
                            drivetrain.backLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f + increment);
                    break;
                case "frontRight":
                    drivetrain.frontRight.setVelocityPIDFCoefficients( 0, 0, 0,
                            drivetrain.frontRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f + increment);
                    break;
                case "frontLeft":
                    drivetrain.frontLeft.setVelocityPIDFCoefficients( 0, 0, 0,
                            drivetrain.frontLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f + increment);
                    break;
            }
        }
        else if( driverOI.x.wasJustPressed() )
            increment *= -1;
        else if( driverOI.back.wasJustPressed() )
            increment = 0.1;
        if( driverOI.left_trigger.get() > 0.1 )
            drivetrain.driveTest(.75);
        else if( driverOI.right_trigger.get() > 0.1 )
            drivetrain.driveTest(-.75);
        else
            drivetrain.driveTest(0.0);

        // Show Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Increment", increment);
        telemetry.addData("Which motor", motor);
        telemetry.addData("backLeft F", drivetrain.backLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        telemetry.addData("backRight F", drivetrain.backRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        telemetry.addData("frontLeft F", drivetrain.frontLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        telemetry.addData("frontRight F", drivetrain.frontRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        double[] motorSpeeds = drivetrain.getMotorVelocities();
        telemetry.addData("BackLeft Vel", motorSpeeds[0]);
        telemetry.addData("BackRight Vel", motorSpeeds[1]);
        telemetry.addData("FrontLeft Vel", motorSpeeds[2]);
        telemetry.addData("FrontRight Vel", motorSpeeds[3]);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}