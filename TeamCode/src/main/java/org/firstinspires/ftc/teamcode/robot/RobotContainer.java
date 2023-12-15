package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controller.GameController;
import org.firstinspires.ftc.teamcode.utils.Constants;

//TODO: automated handoff in the robot
//TODO: lights change with forbar mode since it's toggle, and all other toggles?
//TODO: add all controller functionalities
//TODO: test and troubleshoot full blue AND RED alliance controls
//TODO: localization/spline demo teleop for judging room
//TODO: autos

/**
 * The Robot Container. Contains the robot.
 */
public class RobotContainer implements Constants {
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
    private double splineScoringY;
    private boolean fourbar;

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
        this.fourbar = true;

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

        driverControls();
        operatorControls();

        setLightsColor();
        printTelemetry();
    }

    private void updateInstances() {
        driverOI.updateValues();
        operatorOI.updateValues();

        drivetrain.updateWithOdometry();

        if(!driverOI.start.get() && !operatorOI.start.get())
            return;
        double[] pose = vision.update();
        if(pose != null)
            drivetrain.updateWithAprilTags(pose);
    }

    private void driverControls() {
        intake.setPanPos(!(driverOI.right_trigger.get() > 0) );

        if(driverOI.left_bumper.get() )
            arm.setExtensionPos(1.0);

        power = driverOI.leftStickRadius();
        angle = driverOI.leftStickTheta(alliance);
        turn = driverOI.right_stick_x.get();

        if(driverOI.left_trigger.get() > 0.0) {
            power *= VIRTUAL_LOW_GEAR;
            turn *= VIRTUAL_LOW_GEAR;
        }

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

        if(alliance)
            blueSplineControls();
        else
            redSplineControls();

        if(power != 0.0) {
            splineToScoring = false;
            splineToIntake = false;
        }

        if(splineToIntake)
            drivetrain.splineToIntake(turn, autoAlign);
        else if(splineToScoring)
            drivetrain.splineToScoring(turn, autoAlign, splineScoringY);
        else
            drivetrain.drive(power, angle, turn, autoAlign, fieldOriented);
    }

    private void blueSplineControls() {
        if(driverOI.dpad_left.wasJustPressed()) {
            splineScoringY = BLUE_SCORING_Y_MED;
            splineToScoring = true;
        }
        else if(driverOI.dpad_right.wasJustPressed())
            splineToIntake = true;

        else if(driverOI.dpad_up.wasJustPressed()) {
            splineScoringY = BLUE_SCORING_Y_FAR;
            splineToScoring = true;
        }
        else if(driverOI.dpad_down.wasJustPressed()) {
            splineScoringY = BLUE_SCORING_Y_CLOSE;
            splineToScoring = true;
        }
    }

    private void redSplineControls() {
        if(driverOI.dpad_left.wasJustPressed())
            splineToIntake = true;

        else if(driverOI.dpad_right.wasJustPressed()) {
            splineScoringY = RED_SCORING_Y_MED;
            splineToScoring = true;
        }
        else if(driverOI.dpad_up.wasJustPressed()) {
            splineScoringY = RED_SCORING_Y_FAR;
            splineToScoring = true;
        }
        else if(driverOI.dpad_down.wasJustPressed()) {
            splineScoringY = RED_SCORING_Y_CLOSE;
            splineToScoring = true;
        }
    }

    private void operatorControls() {
        if(operatorOI.b.wasJustPressed())
            fourbar = true;

        if(operatorOI.right_stick_y.get() != 0.0) {
            fourbar = false;
            arm.wristManualControl(operatorOI.right_stick_y.get());
        }
        else if(fourbar)
            arm.wristFourbar();

        double armDesiredAngle;
        if(operatorOI.dpad_up.get())
            armDesiredAngle = 180.0;
        else if(operatorOI.dpad_down.get())
            armDesiredAngle = -15.0;
        else if(operatorOI.dpad_left.get() && alliance || operatorOI.dpad_right.get() && !alliance)
            armDesiredAngle = 90.0;
        else if(operatorOI.dpad_left.get() && !alliance || operatorOI.dpad_right.get() && alliance)
            armDesiredAngle = LAUNCH_ARM_ANGLE;
        else
            armDesiredAngle = arm.getArmAngle();

        if(operatorOI.left_stick_y.get() == 0.0)
            arm.armGoToPos(armDesiredAngle);
        else
            arm.armManualControl(operatorOI.left_stick_y.get());

        if(operatorOI.a.get())
            hand.setClawPos(1.0);
        else if(operatorOI.x.get())
            hand.setClawPos(0.5);

        hand.setTurnyWristPos(0.5); //TODO: this functionality

        if(operatorOI.right_bumper.get())
            arm.setExtensionPos(0.7);
        else if(operatorOI.left_bumper.get())
            arm.setExtensionPos(0.0);

        if(operatorOI.left_trigger.get() > 0.0)
            arm.winchRobot();
        else if(operatorOI.right_trigger.get() > 0.0)
            arm.winchOpposite();
        else
            arm.stopWinch();

        if(operatorOI.back.get())
            arm.setPlaneLauncher(true);
    }

    private void handoff() {
        //TODO: this too
    }

    private void setLightsColor() {
        if(!fieldOriented)
            lights.setColour(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
        else if(splineToScoring || splineToIntake)
            lights.setColour(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        else
            lights.setDefaultColor();
    }

    private void printTelemetry() {
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
