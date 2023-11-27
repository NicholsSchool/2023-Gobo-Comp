package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controller.GameController;

//TODO: double cameras
//TODO: optimize exposure
//TODO: camera localization
//TODO: re-tune odometry
//TODO: re-tune drive motors
//TODO: arm stuff
//TODO: test full blue alliance controls
//TODO: test full red alliance controls

/**
 * The Robot Container
 */
public class RobotContainer {
    private boolean alliance;
    private Drivetrain drivetrain;
    private Intake intake;
    private Arm arm;
    private IndicatorLights lights;
    private GameController driverOI;
    private GameController operatorOI;
    private Telemetry telemetry;
    private double power;
    private double angle;
    private double turn;
    private boolean fieldOriented;
    private boolean splineToIntake;
    private boolean splineToScoring;

    /**
     * Initialize the RobotContainer object
     *
     * @param hwMap the hardWareMap
     * @param telemetry the telemetry
     * @param alliance  the alliance
     * @param x starting x
     * @param y starting y
     */
    public void init(HardwareMap hwMap, Telemetry telemetry, boolean alliance, double x, double y, Gamepad g1, Gamepad g2) {
        this.alliance = alliance;
        drivetrain = new Drivetrain();
        intake = new Intake();
        arm = new Arm();
        lights = new IndicatorLights();
        drivetrain.init(hwMap, alliance, x, y);
        intake.init(hwMap);
        arm.init(hwMap);
        lights.init(hwMap, alliance);
        driverOI = new GameController(g1);
        operatorOI = new GameController(g2);
        this.telemetry = telemetry;
        power = 0.0;
        angle = 0.0;
        turn = 0.0;
        fieldOriented = true;
        splineToIntake = false;
        splineToScoring = false;
    }

    /**
     * Robots. Call in each loop() of the teleop.
     */
    public void robot() {
        updateInstances();

        if(driverOI.left_trigger.get() > 0.0)
            intake.panToPosition(false);
        else if(driverOI.right_trigger.get() > 0.0)
            intake.panToPosition(true);

        power = driverOI.leftStickRadius();
        angle = driverOI.leftStickTheta(alliance);
        turn = driverOI.right_stick_x.get();

        if(driverOI.back.wasJustPressed())
            fieldOriented = !fieldOriented;

        if(!driverOI.right_stick_x.hasBeenZeroForEnoughTime() )
            drivetrain.setDesiredHeading(drivetrain.getFieldHeading());
        else if(driverOI.y.wasJustPressed())
            drivetrain.setDesiredHeading(alliance ? 90.0 : -90.0);
        else if(driverOI.a.wasJustPressed())
            drivetrain.setDesiredHeading(alliance ? -90.0 : 90.0);
        else if(driverOI.b.wasJustPressed())
            drivetrain.setDesiredHeading(alliance ? 0.0 : -180.0);
        else if(driverOI.x.wasJustPressed())
            drivetrain.setDesiredHeading(alliance ? -180.0 : 0.0);

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
            drivetrain.drive(power, angle, turn, driverOI.right_stick_x.hasBeenZeroForEnoughTime(), fieldOriented);

        setLightsColor();

        printTelemetry();
    }

    /**
     * Updates Objects such as Gamepads and Subsystems
     */
    public void updateInstances() {
        driverOI.updateValues();
        operatorOI.updateValues();

        drivetrain.updatePose();
    }

    /**
     * Sets the light patterns for the robot
     */
    public void setLightsColor() {
        if(!fieldOriented)
            lights.setColour(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
        else if(splineToScoring || splineToIntake)
            lights.setColour(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        else
            lights.setDefaultColor();
    }

    /**
     * Prints Telemetry to the Driver Station
     */
    public void printTelemetry() {
        telemetry.addData("Drive Power", power);
        telemetry.addData("angle", angle);
        telemetry.addData("turn power", turn);
        double[] motorVel = drivetrain.getMotorVelocities();
        telemetry.addData("back left drive vel", motorVel[0]);
        telemetry.addData("back right drive vel", motorVel[1]);
        telemetry.addData("front left drive vel", motorVel[2]);
        telemetry.addData("front right drive vel", motorVel[3]);
        double[] deadPos = drivetrain.getOdometryPositions();
        telemetry.addData("left dead pos", deadPos[0]);
        telemetry.addData("right dead pos", deadPos[1]);
        telemetry.addData("center dead pos", deadPos[2]);
        double[] xy = drivetrain.getXY();
        telemetry.addData("x", xy[0]);
        telemetry.addData("y", xy[1]);
        telemetry.addData("heading", drivetrain.getFieldHeading());
        telemetry.addData("autoAligning", turn == 0.0);
        telemetry.addData("field oriented", fieldOriented);
        telemetry.addData("pot", arm.getPot());
        telemetry.update();
    }
}
