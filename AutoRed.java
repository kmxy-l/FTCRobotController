package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.Configuration.liftBottom;
import static org.firstinspires.ftc.teamcode.Configuration.liftTop;
import static org.firstinspires.ftc.teamcode.Configuration.backLauncherPollenSpeed;
import static org.firstinspires.ftc.teamcode.Configuration.middleLauncherPollenSpeed;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "RedAuto", group = "RedAuto")
public class RedAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(9.1392, 33.1646, 90);
    private final Pose point1 = poseFactory.of(58.0316, 10.9948, -89.0938);
    private final Pose point1Control1 = poseFactory.of(57.6826, 33.0634, 0);
    private final Pose point2 = poseFactory.of(9.914, 94.8096, 0);
    private final Pose point2Control1 = poseFactory.of(26.1702, 18.6796, 0);
    private final Pose point2Control2 = poseFactory.of(26.7455, 95.168, 0);
    private final Pose point2Segment1Start = poseFactory.of(9.914, 94.8096, -90);
    private final Pose point2Segment1End = poseFactory.of(9.914, 94.8096, -90);
    private final Pose point2Segment2Start = poseFactory.of(9.914, 94.8096, -90);
    private final Pose point2Segment2End = poseFactory.of(9.914, 94.8096, 0);


    //private DcMotor intake;
    private DcMotorEx middleLauncher;
    private DcMotorEx backLauncher;
    private Servo lift;


    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                launchSequence(),
                follow(follower, path2())
        );
    }
    private Command launchSequence() {
        return sequential(
                instant(() -> lift.setPosition(liftTop)),
                instant(() -> backLauncher.setVelocity(backLauncherPollenSpeed)),
                instant(() -> middleLauncher.setVelocity(middleLauncherPollenSpeed)),

                waitMs(1000),

                instant(() -> lift.setPosition(liftBottom)),
                waitMs(1000),
                instant(() -> lift.setPosition(liftTop)),

                waitMs(1000),
                instant(() -> lift.setPosition(liftBottom)),
                waitMs(1000),
                instant(() -> lift.setPosition(liftTop)),

                waitMs(1000),
                instant(() -> lift.setPosition(liftBottom)),
                waitMs(1000),
                instant(() -> lift.setPosition(liftTop)),

                instant(() -> backLauncher.setVelocity(0)),
                instant(() -> middleLauncher.setVelocity(0))
        );
    }
    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        //intake = hardwareMap.get(DcMotor.class, "Intake");
        middleLauncher = hardwareMap.get(DcMotorEx.class, "MiddleLauncher");
        backLauncher = hardwareMap.get(DcMotorEx.class, "BackLauncher");
        lift = hardwareMap.get(Servo.class, "lift");
        CRServo spinner = hardwareMap.get(CRServo.class, "spinner");

        middleLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setPosition(liftTop);
        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            spinner.setPower(1);

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.curve(start, point1Control1, point1).tangent();
    }

    public Path path2() {
        return Paths.curve(point1, point2Control1, point2Control2, point2).heading(Interpolator.piecewise().until(0.351, Interpolator.linear(point2Segment1Start, point2Segment1End)).until(1, Interpolator.linear(point2Segment2Start, point2Segment2End)));
    }
}
