package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.commands.Commands.*;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
@Autonomous(name = "Auto Red")
public class AutoRed extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(8.7003, 106.9901, 90);
    private final Pose path1 = poseFactory.of(25.2652, 13.4478, -94.6474);
    private final Pose path1Control1 = poseFactory.of(30.8597, 80.7666, 0);
    private final Pose point2Start = poseFactory.of(25.2652, 13.4478, 0);
    private final Pose point2 = poseFactory.of(58.4774, 13.2694, 90);
    public DcMotor backLeft, backRight, frontLeft, frontRight, intake, middleLauncher, backLauncher;
    public Servo lift;


    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                instant(() -> launchSequence()),
                waitMs(1000),

                instant(() -> launchSequence()),
                waitMs(1000),

                instant(() -> launchSequence())
        );
    }
    public Path path1() {
        return Paths.curve(start, path1Control1, path1).tangent();
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }
    public void launchSequence() {
        middleLauncher.setPower(0.75);
        backLauncher.setPower(0.6);
        sleep(1000);
        lift.setPosition(0.5);
        sleep(500);
        lift.setPosition(0);

    }

    @Override
    public void runOpMode() {

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

         backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
         backRight = hardwareMap.get(DcMotor.class, "BackRight");
         frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
         frontRight = hardwareMap.get(DcMotor.class, "FrontRight");

         intake = hardwareMap.get(DcMotor.class, "Intake");

         middleLauncher = hardwareMap.get(DcMotor.class, "MiddleLauncher");
         backLauncher = hardwareMap.get(DcMotor.class, "BackLauncher");

         lift = hardwareMap.get(Servo.class, "Lift");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        backLauncher.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        schedule(autoRoutine());

        if (opModeIsActive()) {

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

                telemetry.update();
            }
        }
    }
}
