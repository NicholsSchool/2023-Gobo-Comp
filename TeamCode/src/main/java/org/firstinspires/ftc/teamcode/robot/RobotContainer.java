package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controller.GameController;
import org.firstinspires.ftc.teamcode.utils.Constants;


//TODO: add the new functionalities to controllers
//TODO: test and troubleshoot full blue AND red alliance controls
//TODO: automated handoff in the robot

/**
 * The Robot Container
 */
public class RobotContainer implements Constants{
    private final boolean alliance;
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Arm arm;
    private final Hand hand;
    private final IndicatorLights lights;
    private final Vision vision;
    private final GameController driverOI;
    private final GameController operatorOI;
    private final Telemetry telemetry;
    private double power;
    private double angle;
    private double turn;
    private boolean fieldOriented;
    private boolean autoAlign;
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
    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, boolean alliance, double x, double y, double heading, Gamepad g1, Gamepad g2) {
        this.alliance = alliance;
        this.fieldOriented = true;
        this.autoAlign = true;

        drivetrain = new Drivetrain(hwMap, alliance, x, y, heading);
        intake = new Intake(hwMap);
        arm = new Arm(hwMap);
        hand = new Hand(hwMap);
        lights = new IndicatorLights(hwMap, alliance);
        vision = new Vision(hwMap);

        driverOI = new GameController(g1);
        operatorOI = new GameController(g2);

        this.telemetry = telemetry;
    }

    /**
     * Robots. Call in each loop() of the teleop.
     */
    public void robot() {
        updateInstances();

        arm.armManualControl(operatorOI.left_stick_y.get());

        intake.setPanPos(!(driverOI.left_trigger.get() > 0) );

        power = driverOI.leftStickRadius();
        angle = driverOI.leftStickTheta(alliance);
        turn = driverOI.right_stick_x.get();

        autoAlign = driverOI.right_stick_x.wasZeroLongEnough();

        if(!autoAlign)
            drivetrain.setDesiredHeading(drivetrain.getFieldHeading());
        else if(driverOI.y.wasJustPressed())
            drivetrain.setDesiredHeading(alliance ? 90.0 : -90.0);
        else if(driverOI.a.wasJustPressed())
            drivetrain.setDesiredHeading(alliance ? -90.0 : 90.0);
        else if(driverOI.b.wasJustPressed())
            drivetrain.setDesiredHeading(alliance ? 0.0 : -180.0);
        else if(driverOI.x.wasJustPressed())
            drivetrain.setDesiredHeading(alliance ? -180.0 : 0.0);

        fieldOriented = !driverOI.back.getToggleState();

        if(driverOI.dpad_left.wasJustPressed())
            splineToScoring = true;
        else if(driverOI.dpad_right.wasJustPressed())
            splineToIntake = true;

        if(power != 0.0) {
            splineToScoring = false;
            splineToIntake = false;
        }

        if(splineToIntake)
            drivetrain.splineToIntake(turn, autoAlign);
        else if(splineToScoring)
            drivetrain.splineToScoring(turn, autoAlign);
        else
            drivetrain.drive(power, angle, turn, autoAlign, fieldOriented);

        setLightsColor();
        printTelemetry();
    }

    /**
     * Updates GameControllers and Robot Pose
     */
    public void updateInstances() {
        driverOI.updateValues();
        operatorOI.updateValues();

        drivetrain.updateWithOdometry();
        double[] pose = vision.update();
        if(pose != null && driverOI.start.get() )
            drivetrain.updateWithAprilTags(pose);
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
        telemetry.addData("autoAligning", autoAlign);
        telemetry.addData("field oriented", fieldOriented);
        telemetry.addData("pot", arm.getPot());
        telemetry.addData("April Tag Detections", vision.getNumDetections() );

        telemetry.update();
    }
}
