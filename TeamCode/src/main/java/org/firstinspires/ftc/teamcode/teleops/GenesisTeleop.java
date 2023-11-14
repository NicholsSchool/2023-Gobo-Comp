package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.GameController;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;
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
    private RobotContainer robotContainer;
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
        robotContainer = new RobotContainer();
        robotContainer.init(hardwareMap, alliance, 0.0, 0.0);
        fieldOriented = true;
        splineToIntake = false;
        splineToScoring = false;
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
        updateInstances();

        if(driverOI.left_trigger.get() > 0.0)
            robotContainer.intake.panToPosition(false);
        else if(driverOI.right_trigger.get() > 0.0)
            robotContainer.intake.panToPosition(true);

        power = driverOI.leftStickRadius();
        angle = driverOI.leftStickTheta(alliance);
        turn = driverOI.right_stick_x.get();

        if(driverOI.start.wasJustPressed())
            robotContainer.drivetrain.setHeadingOffset();

        if(driverOI.back.wasJustPressed())
            fieldOriented = !fieldOriented;

        if(turn != 0.0)
            robotContainer.drivetrain.setDesiredHeading(robotContainer.drivetrain.getFieldHeading());
        else if(driverOI.y.wasJustPressed())
            robotContainer.drivetrain.setDesiredHeading(90.0);
        else if(driverOI.a.wasJustPressed())
            robotContainer.drivetrain.setDesiredHeading(-90.0);
        else if(driverOI.b.wasJustPressed())
            robotContainer.drivetrain.setDesiredHeading(0.0);
        else if(driverOI.x.wasJustPressed())
            robotContainer.drivetrain.setDesiredHeading(-180.0);

        if(driverOI.dpad_left.wasJustPressed())
            splineToScoring = true;
        else if(driverOI.dpad_right.wasJustPressed())
            splineToIntake = true;

        if(power != 0.0) {
            splineToScoring = false;
            splineToIntake = false;
        }

//        if(splineToIntake)
//            robotContainer.drivetrain.splineToIntake(turn, true);
//        else if(splineToScoring)
//            robotContainer.drivetrain.splineToScoring(turn, true);
//        else
            robotContainer.drivetrain.drive(power, angle, turn, turn == 0, fieldOriented);

        printTelemetry();
    }

    /**
     * Updates Objects such as Gamepads and Subsystems
     */
    public void updateInstances() {
        driverOI.updateValues();
        operatorOI.updateValues();

        robotContainer.drivetrain.updatePose();
    }

    /**
     * Prints Telemetry to the Driver Station
     */
    public void printTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Drive Power", power);
        telemetry.addData("angle", angle);
        telemetry.addData("turn power", turn);
//        double[] motorPos = drivetrain.getMotorPositions();
//        telemetry.addData("back left drive pos", motorPos[0]);
//        telemetry.addData("back right drive pos", motorPos[1]);
//        telemetry.addData("front left drive pos", motorPos[2]);
//        telemetry.addData("front right drive pos", motorPos[3]);
        double[] motorVel = robotContainer.drivetrain.getMotorVelocities();
        telemetry.addData("back left drive vel", motorVel[0]);
        telemetry.addData("back right drive vel", motorVel[1]);
        telemetry.addData("front left drive vel", motorVel[2]);
        telemetry.addData("front right drive vel", motorVel[3]);
        double[] deadPos = robotContainer.drivetrain.getOdometryPositions();
        telemetry.addData("left dead pos", deadPos[0]);
        telemetry.addData("right dead pos", deadPos[1]);
        telemetry.addData("center dead pos", deadPos[2]);
        double[] xy = robotContainer.drivetrain.getXY();
        telemetry.addData("x", xy[0]);
        telemetry.addData("y", xy[1]);
        telemetry.addData("heading", robotContainer.drivetrain.getFieldHeading());
        telemetry.addData("raw imu", robotContainer.drivetrain.getRawHeading());
        telemetry.addData("autoAligning", turn == 0.0);
        telemetry.addData("field oriented", fieldOriented);
        telemetry.addData("pot", robotContainer.arm.getPot());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
