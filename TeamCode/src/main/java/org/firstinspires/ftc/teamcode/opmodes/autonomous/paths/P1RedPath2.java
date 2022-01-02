/*
blue path 2
starts warehouse blue
delivers cube to hub
back to wall
strafe to warehouse
move to 2nd square in warehouse
 */
package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmLevel;
import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.webcam.DetectTSEPosition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateArm;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateIntake;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateWebCam;
import org.firstinspires.ftc.teamcode.subsystems.drive.roadrunner.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

public class P1RedPath2 {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand sample1Follower1;
    private TrajectoryFollowerCommand sample1Follower2;

    private FtcDashboard dashboard;


    private Pose2d startPose;
    private final HardwareMap hwMap;
    private final Telemetry telemetry;

    public P1RedPath2(HardwareMap hwMap, Telemetry telemetry){
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hwMap), false);

    }

    public P1RedPath2(HardwareMap hwMap, FtcDashboard db, Telemetry telemetry){
        this.hwMap = hwMap;
        dashboard = db;
        this.telemetry = telemetry;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hwMap), false);

    }

    public void createPath(){
        startPose = new Pose2d(-36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        CreateWebCam createWebCam = new CreateWebCam(hwMap, "Webcam 1", dashboard, telemetry);
        CreateArm createArm = new CreateArm(hwMap, "arm", telemetry);

        createArm.createAuto();

        createWebCam.createAuto();
        WebCamSubsystem webCamSubsystem = createWebCam.getWebCamSubsystem();

        //MockDetectTSEPosition mockDetectTSEPosition = createWebCam.getMockDetectTSEPositionCommand();
        //mockDetectTSEPosition.schedule();

        DetectTSEPosition detectTSEPosition = createWebCam.getDetectTSEPositionCommand();
        detectTSEPosition.schedule();


        CreateIntake createIntake = new CreateIntake(hwMap, "intake", telemetry);
        createIntake.createAuto();


        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-12, -42))
                .addDisplacementMarker(()->{
                    SetArmLevel setArmLevel = createArm.createSetArmLevel(webCamSubsystem.getLevel());
                    setArmLevel.schedule();
                })
                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .addDisplacementMarker(()->{

                    createIntake.getSeGrabber().schedule();
                    new WaitCommand(800)
                            .andThen(createIntake.getStopIntake()).schedule();
                })
                .strafeTo(new Vector2d(-12, -64))
                .strafeTo(new Vector2d(44, -64))
                .strafeTo(new Vector2d(44, -40))
                .build();


        sample1Follower1 = new TrajectoryFollowerCommand(drive,traj1);
        sample1Follower2 = new TrajectoryFollowerCommand(drive,traj2);

    }

    public void execute(CommandOpMode commandOpMode){
        commandOpMode.schedule(new WaitUntilCommand(commandOpMode::isStarted).andThen(
                sample1Follower1.andThen(sample1Follower2)
        ));
    }
}
