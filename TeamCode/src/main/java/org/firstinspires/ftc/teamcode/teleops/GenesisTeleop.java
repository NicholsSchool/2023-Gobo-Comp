package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.GameController;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.utils.Constants;

@TeleOp(name = "Teleop: Robot Genesis")
public class GenesisTeleop extends OpMode implements Constants {
    // Declare OpMode members.
    private boolean alliance;
    private double power;
    private double angle;
    private double turn;
    private final ElapsedTime runtime = new ElapsedTime();
    private GameController driverOI;
    private GameController operatorOI;
    private Drivetrain drivetrain;
    private Intake intake;
    private boolean fieldOriented;
    private boolean splineToIntake;
    private boolean splineToScoring;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        alliance = BLUE_ALLIANCE;
        power = 0.0;
        angle = 0.0;
        turn = 0.0;
        driverOI = new GameController(gamepad1);
        operatorOI = new GameController(gamepad2);
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap, alliance, 0.0, 0.0);
        intake = new Intake();
        intake.init(hardwareMap);
        fieldOriented = true;
        splineToIntake = false;
        splineToScoring = false;

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

        drivetrain.updatePose();

        if(driverOI.left_trigger.get() > 0.0)
            intake.panToPosition(false);
        else if(driverOI.right_trigger.get() > 0.0)
            intake.panToPosition(true);

        power = driverOI.leftStickRadius();
        angle = driverOI.leftStickTheta(alliance);
        turn = driverOI.right_stick_x.get();

        if(driverOI.start.wasJustPressed())
            drivetrain.setHeadingOffset();

        if(driverOI.back.wasJustPressed())
            fieldOriented = !fieldOriented;

        if(turn != 0.0)
            drivetrain.setDesiredHeading(drivetrain.getFieldHeading());
        else if(driverOI.y.wasJustPressed())
            drivetrain.setDesiredHeading(90.0);
        else if(driverOI.a.wasJustPressed())
            drivetrain.setDesiredHeading(-90.0);
        else if(driverOI.b.wasJustPressed())
            drivetrain.setDesiredHeading(0.0);
        else if(driverOI.x.wasJustPressed())
            drivetrain.setDesiredHeading(-180.0);

        if(driverOI.dpad_left.wasJustPressed())
            splineToScoring = true;
        else if(driverOI.dpad_right.wasJustPressed())
            splineToIntake = true;

        if(power != 0.0) {
            splineToScoring = false;
            splineToIntake = false;
        }

        if(splineToIntake)
            drivetrain.splineToIntake(turn, true);
        else if(splineToScoring)
            drivetrain.splineToScoring(turn, true);
        else
            drivetrain.drive(power, angle, turn, turn == 0.0, fieldOriented);

        updateTelemetry();
    }

    /**
     * Prints Telemetry to the Driver Station
     */
    public void updateTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Drive Power", power);
        telemetry.addData("angle", angle);
        telemetry.addData("turn power", turn);
//        double[] motorPos = drivetrain.getMotorPositions();
//        telemetry.addData("back left drive pos", motorPos[0]);
//        telemetry.addData("back right drive pos", motorPos[1]);
//        telemetry.addData("front left drive pos", motorPos[2]);
//        telemetry.addData("front right drive pos", motorPos[3]);
        double[] motorVel = drivetrain.getMotorVelocities();
        telemetry.addData("back left drive vel", motorVel[0]);
        telemetry.addData("back right drive vel", motorVel[1]);
        telemetry.addData("front left drive vel", motorVel[2]);
        telemetry.addData("front right drive vel", motorVel[3]);
//        double[] deadPos = drivetrain.getOdometryPositions();
//        telemetry.addData("left dead pos", deadPos[0]);
//        telemetry.addData("right dead pos", deadPos[1]);
//        telemetry.addData("center dead pos", deadPos[2]);
        double[] xy = drivetrain.getXY();
        telemetry.addData("x", xy[0]);
        telemetry.addData("y", xy[1]);
        telemetry.addData("imu", drivetrain.getFieldHeading());
        telemetry.addData("autoAligning", turn == 0.0);
        telemetry.addData("field oriented", fieldOriented);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
