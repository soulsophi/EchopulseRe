//package org.firstinspires.ftc.teamcode.autonomous.opmode;
//
//import android.util.Size;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.MinMax;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Twist2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.smartcluster.meepmeep.AutonomousRedTrussPoses;
//import com.smartcluster.meepmeep.AutonomousUtils;
//import com.smartcluster.oracleftc.commands.Command;
//import com.smartcluster.oracleftc.commands.CommandScheduler;
//import com.smartcluster.oracleftc.commands.helpers.InstantCommand;
//import com.smartcluster.oracleftc.commands.helpers.ParallelCommand;
//import com.smartcluster.oracleftc.commands.helpers.RaceCommand;
//import com.smartcluster.oracleftc.commands.helpers.SequentialCommand;
//import com.smartcluster.oracleftc.commands.helpers.WaitCommand;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.autonomous.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.autonomous.vision.CaseDetectionVisionProcessor;
//import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
//import org.firstinspires.ftc.teamcode.utils.Kinematics;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import java.util.HashMap;
//import java.util.List;
//import java.util.concurrent.atomic.AtomicBoolean;
//import java.util.concurrent.atomic.AtomicReference;
//
//@Autonomous
//public class AutonomousRedTruss extends LinearOpMode {
//    private CommandScheduler scheduler;
//    private final AutonomousUtils.AllianceColor color= AutonomousUtils.AllianceColor.Red;
//
//    private final AutonomousUtils.Park park= AutonomousUtils.Park.Center;
//    private static Action commandToAction(Command c)
//    {
//        return new Action() {
//            private boolean initialized=false;
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if(!initialized) {
//                    c.init();
//                    initialized = true;
//                }
//                c.update();
//                if(c.finished())
//                {
//                    c.end(false);
//                    return false;
//                }else return true;
//            }
//        };
//    }
//    long lastTime=-1;
//
//    private IntakeSubsystem intakeSubsystem;
//    private DepositSubsystem depositSubsystem;
//    private LiftSubsystem liftSubsystem;
//    private SliderSubsystem sliderSubsystem;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        AutonomousUtils.StartPosition startPosition = AutonomousUtils.StartPosition.Truss;
//        scheduler = new CommandScheduler();
//        CaseDetectionVisionProcessor processor = new CaseDetectionVisionProcessor(color, startPosition);
//        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, AutonomousRedTrussPoses.getStartPose(color, startPosition));
//        depositSubsystem = new DepositSubsystem(this);
//        intakeSubsystem = new IntakeSubsystem(this);
//        liftSubsystem = new LiftSubsystem(this);
//        sliderSubsystem = new SliderSubsystem(this);
//
//        WebcamName c270 = hardwareMap.get(WebcamName.class, "C270");
////        WebcamName arducam = hardwareMap.get(WebcamName.class, "Arducam");
////        SwitchableCameraName switchableCamera = ClassFactory.getInstance()
////                .getCameraManager().nameForSwitchableCamera(c270, arducam);
//
//        VisionPortal portal = new VisionPortal.Builder()
//                .setCamera(c270)
//                .setCameraResolution(new Size(1280, 720))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .setAutoStopLiveView(true)
//                .addProcessor(processor)
////                .addProcessor(frontProcessor)
//                .build();
//        AutonomousUtils.AutoCase autoCase = null;
//
////        portal.setProcessorEnabled(frontProcessor, false);
//        Command.run(new SequentialCommand(
//                depositSubsystem.reset(),
//                liftSubsystem.reset(),
//                sliderSubsystem.reset(),
//                new InstantCommand(sliderSubsystem::resetPitch),
//                intakeSubsystem.lockColor(color),
//                new RaceCommand(
//                        depositSubsystem.update(),
//                        depositSubsystem.pitch(new AtomicReference<>(13.86))
//                )
//        ));
//        scheduler.schedule(new ParallelCommand(
//                depositSubsystem.update(),
//                liftSubsystem.update(),
//                sliderSubsystem.update()
//        ));
//
//        HashMap<AutonomousUtils.AutoCase, Action> auto = new HashMap<>();
//        double[] firstRowKinematics = Kinematics.inverseKinematics(390, 260);
//        double[] safeRowKinematics = Kinematics.inverseKinematics(390, 260);
//        double[] secondRowKinematics = Kinematics.inverseKinematics(390, 260);
//        for (AutonomousUtils.AutoCase i : AutonomousUtils.AutoCase.values()) {
//            auto.put(i, new SequentialAction(
//                    new ParallelAction(
//                            caseAndStack(mecanumDrive, color, i),
//                            commandToAction(
//                                    new SequentialCommand(
//                                            intakeSubsystem.reset(),
//                                            intakeSubsystem.lockColor(color),
//                                            new WaitCommand(()->Math.abs(mecanumDrive.pose.position.y)< AutonomousUtils.TILE_WIDTH),
//                                            new WaitCommand(500),
//                                            intakeSubsystem.stack(4)
//                                    )
//                            )
//                    ),
//                    oscillateUntilIntake(mecanumDrive,color),
//                    new ParallelAction(
//                            stackToBackdrop(mecanumDrive, color, startPosition, i),
//                            commandToAction(
//                                    new ParallelCommand(
//                                            new SequentialCommand(
//                                                    new WaitCommand(300),
//                                                    intakeSubsystem.outtake(),
//                                                    new WaitCommand(500),
//                                                    intakeSubsystem.stop(),
//                                                    intakeSubsystem.lift(),
//                                                    //sliderSubsystem.releaseContact()
//                                                    intakeSubsystem.unlockGrippers(),
//                                                    intakeSubsystem.outtake(),
//                                                    new WaitCommand(800),
//                                                    intakeSubsystem.stop()
//                                                    //sliderSubsystem.releaseContact()
//                                            ),
//                                            new SequentialCommand(
//                                                    new WaitCommand(()->mecanumDrive.pose.position.x> AutonomousUtils.TILE_WIDTH&&depositSubsystem.pixelsInBucket()&&sliderSubsystem.updateMode!= SliderSubsystem.UpdateMode.ENSURE_CONTACT),
//                                                    new ParallelCommand(
//                                                            sliderSubsystem.move(new AtomicReference<>(firstRowKinematics[0])),
//                                                            liftSubsystem.move(new AtomicReference<>(firstRowKinematics[1])),
//                                                            depositSubsystem.pitch(new AtomicReference<>(firstRowKinematics[2]))
//                                                    )
//                                            )
//                                    )
//
//                            )
//
//                    ),
//                    commandToAction(
//                            new SequentialCommand(
//                                    new WaitCommand(300),
//                                    depositSubsystem.open(),
//                                    new ParallelCommand(
//                                            sliderSubsystem.move(new AtomicReference<>(safeRowKinematics[0])),
//                                            liftSubsystem.move(new AtomicReference<>(safeRowKinematics[1])),
//                                            depositSubsystem.pitch(new AtomicReference<>(safeRowKinematics[2]))
//                                    ),
//                                    new WaitCommand(500)
//
//                            )
//                    ),
//                    new ParallelAction(
//                            commandToAction(
//                                    new ParallelCommand(
//                                            new SequentialCommand(
//                                                    new WaitCommand(500),
//                                                    sliderSubsystem.move(new AtomicReference<>(0.0))
//                                            ),
//                                            new SequentialCommand(
//                                                    new WaitCommand(250),
//                                                    depositSubsystem.pitch(new AtomicReference<>(13.86)),
//                                                    depositSubsystem.close()
//                                            ),
//                                            new SequentialCommand(
//                                                    new WaitCommand(750),
//                                                    liftSubsystem.move(new AtomicReference<>(0.0))
//                                            )
//                                    )
//                            ),
//                            commandToAction(
//                                    new SequentialCommand(
//                                            new WaitCommand(()->mecanumDrive.pose.position.x<-AutonomousUtils.TILE_WIDTH_HALF),
//                                            intakeSubsystem.stack(2)
//                                    )
//                            ),
//                            backdropToStack(mecanumDrive,color,startPosition,i)
//                    ),
//
//                    oscillateUntilIntake(mecanumDrive,color),
//                    new ParallelAction(
//                            stackToBackdropExterior(mecanumDrive, color, startPosition),
//                            commandToAction(
//                                    new ParallelCommand(
//                                            new SequentialCommand(
//                                                    new WaitCommand(300),
//                                                    intakeSubsystem.outtake(),
//                                                    new WaitCommand(500),
//                                                    intakeSubsystem.stop(),
//                                                    intakeSubsystem.lift(),
//                                                    //sliderSubsystem.releaseContact()
//                                                    intakeSubsystem.unlockGrippers(),
//                                                    intakeSubsystem.outtake(),
//                                                    new WaitCommand(800),
//                                                    intakeSubsystem.stop()
//                                                    //sliderSubsystem.releaseContact()
//                                            ),
//                                            new SequentialCommand(
//                                                    new WaitCommand(()->mecanumDrive.pose.position.x> AutonomousUtils.TILE_WIDTH&&depositSubsystem.pixelsInBucket()&&sliderSubsystem.updateMode!= SliderSubsystem.UpdateMode.ENSURE_CONTACT),
//                                                    new ParallelCommand(
//                                                            sliderSubsystem.move(new AtomicReference<>(secondRowKinematics[0])),
//                                                            liftSubsystem.move(new AtomicReference<>(secondRowKinematics[1])),
//                                                            depositSubsystem.pitch(new AtomicReference<>(secondRowKinematics[2]))
//                                                    )
//                                            )
//                                    )
//
//                            )
//
//                    ),
//                    commandToAction(
//                            new SequentialCommand(
//                                    new WaitCommand(500),
//                                    depositSubsystem.open(),
//                                    new WaitCommand(500)
//                            )
//                    ),
//                    new ParallelAction(
//                            commandToAction(
//                                    new ParallelCommand(
//                                            new SequentialCommand(
//                                                    new WaitCommand(500),
//                                                    sliderSubsystem.move(new AtomicReference<>(0.0))
//                                            ),
//                                            new SequentialCommand(
//                                                    new WaitCommand(250),
//                                                    depositSubsystem.pitch(new AtomicReference<>(13.86)),
//                                                    depositSubsystem.close()
//                                            ),
//                                            new SequentialCommand(
//                                                    new WaitCommand(750),
//                                                    liftSubsystem.move(new AtomicReference<>(0.0))
//                                            )
//                                    )
//                            ),
//                            commandToAction(
//                                    new SequentialCommand(
//                                            new WaitCommand(()->mecanumDrive.pose.position.x<-AutonomousUtils.TILE_WIDTH_HALF),
//                                            intakeSubsystem.stack(4)
//                                    )
//                            ),
//                            backdropToStack2(mecanumDrive,color,startPosition)
//                    ),
//                    oscillateUntilIntake2(mecanumDrive,color),
//                    new ParallelAction(
//                            stack2ToBackdropExterior(mecanumDrive, color, startPosition),
//                            commandToAction(
//                                    new ParallelCommand(
//                                            new SequentialCommand(
//                                                    new WaitCommand(300),
//                                                    intakeSubsystem.outtake(),
//                                                    new WaitCommand(500),
//                                                    intakeSubsystem.stop(),
//                                                    intakeSubsystem.lift(),
//                                                    //sliderSubsystem.releaseContact()
//                                                    intakeSubsystem.unlockGrippers(),
//                                                    intakeSubsystem.outtake(),
//                                                    new WaitCommand(800),
//                                                    intakeSubsystem.stop()
//                                                    //sliderSubsystem.releaseContact()
//                                            ),
//                                            new SequentialCommand(
//                                                    new WaitCommand(()->mecanumDrive.pose.position.x> AutonomousUtils.TILE_WIDTH&&depositSubsystem.pixelsInBucket()&&sliderSubsystem.updateMode!= SliderSubsystem.UpdateMode.ENSURE_CONTACT),
//                                                    new WaitCommand(100),
//                                                    new ParallelCommand(
//                                                            sliderSubsystem.move(new AtomicReference<>(secondRowKinematics[0])),
//                                                            liftSubsystem.move(new AtomicReference<>(secondRowKinematics[1])),
//                                                            depositSubsystem.pitch(new AtomicReference<>(secondRowKinematics[2]))
//                                                    )
//                                            )
//                                    )
//
//                            )
//
//                    ),
//                    commandToAction(
//                            new SequentialCommand(
//                                    new WaitCommand(500),
//                                    depositSubsystem.open(),
//                                    new WaitCommand(500)
//                            )
//                    ),
//                    commandToAction(
//                            new ParallelCommand(
//                                    new SequentialCommand(
//                                            new WaitCommand(500),
//                                            sliderSubsystem.move(new AtomicReference<>(0.0))
//                                    ),
//                                    new SequentialCommand(
//                                            new WaitCommand(250),
//                                            depositSubsystem.pitch(new AtomicReference<>(13.86)),
//                                            depositSubsystem.close()
//                                    ),
//                                    new SequentialCommand(
//                                            new WaitCommand(750),
//                                            liftSubsystem.move(new AtomicReference<>(0.0))
//                                    )
//                            )
//                    )
//            ));
//
//        }
//        while (opModeInInit()) {
//            autoCase = processor.autoCase;
//            telemetry.addData("autoCase", autoCase);
//            telemetry.update();
//        }
//        portal.setProcessorEnabled(processor, false);
//        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule module :
//                modules) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//        AtomicBoolean finishedAction = new AtomicBoolean(false);
//        Actions.runBlocking(
//                new ParallelAction(
//                        commandToAction(Command.builder()
//                                .update(() -> {
//                                    for (LynxModule module :
//                                            modules) {
//                                        module.clearBulkCache();
//                                    }
//                                    try {
//                                        scheduler.update();
//                                    } catch (InterruptedException e) {
//                                        e.printStackTrace();
//                                    }
//                                    long currentTime = System.nanoTime();
//                                    if (lastTime != -1)
//                                        telemetry.addData("loop(hz)", 1000 / ((currentTime - lastTime) / 1E6));
//                                    lastTime = System.nanoTime();
//                                    telemetry.update();
//                                })
//                                .finished(finishedAction::get)
//                                .build()),
//                        new SequentialAction(
//                                auto.get(autoCase),
//                                new InstantAction(() -> finishedAction.set(true))
//                        )
//                )
//        );
//    }
//    public static Action caseAndStack(MecanumDrive drive, AutonomousUtils.AllianceColor color, AutonomousUtils.AutoCase autoCase)
//    {
//        final Pose2d startPose = AutonomousRedTrussPoses.getStartPose(color, AutonomousUtils.StartPosition.Truss);
//        final Pose2d casePose = AutonomousRedTrussPoses.getCasePose(color, AutonomousUtils.StartPosition.Truss,autoCase);
//        final Pose2d stackPose = AutonomousRedTrussPoses.getStackPose(color, AutonomousUtils.StartPosition.Truss);
//        final Twist2d backAway = new Twist2d(new Vector2d(-8.5,0),0);
//
//        switch (autoCase)
//        {
//            case Truss:
//                return drive.actionBuilder(startPose)
//                        .setTangent(startPose.heading)
//                        .splineToLinearHeading(casePose, casePose.heading)
//                        .setTangent(casePose.heading.log()+Math.PI/2)
//                        .setReversed(true)
//                        .splineToLinearHeading(stackPose, AutonomousUtils.mirrorColor(Math.toRadians(90), color))
//                        .build();
//            case Side:
//                return drive.actionBuilder(startPose)
//                        .setTangent(startPose.heading)
//                        .splineToLinearHeading(casePose, casePose.heading)
//                        .setTangent(casePose.heading.log()-Math.PI)
//                        .splineToLinearHeading(casePose.plus(backAway), casePose.heading.log()-Math.PI)
//                        .setTangent(AutonomousUtils.mirrorColor(Math.toRadians(45), color))
//                        .splineToLinearHeading(stackPose, Math.toRadians(180))
//                        .build();
//            case Middle:
//                return drive.actionBuilder(startPose)
//                        .setTangent(startPose.heading)
//                        .splineToLinearHeading(casePose, casePose.heading)
//                        .setTangent(casePose.heading.log()-Math.PI)
//                        .splineToLinearHeading(stackPose, AutonomousUtils.mirrorColor(Math.toRadians(90),color))
//                        .build();
//        }
//        return null;
//    }
//    public Action oscillateUntilIntake(MecanumDrive drive, AutonomousUtils.AllianceColor color)
//    {
//        final Pose2d stackPose = AutonomousRedTrussPoses.getStackPose(color, AutonomousUtils.StartPosition.Truss);
//        final Pose2d nearStackPose = stackPose.plus(new Twist2d(new Vector2d(-7.5,0),0));
//
//        TrajectoryActionBuilder[] builders = new TrajectoryActionBuilder[]
//                {
//                        drive.actionBuilder(stackPose)
//                                .setTangent(Math.toRadians(180))
//                                .setReversed(true)
//                                .splineToLinearHeading(nearStackPose, Math.toRadians(0), ((pose2dDual, posePath, v) -> 30), (pose2dDual, posePath, v) -> new MinMax(-30,30)),
//                        drive.actionBuilder(nearStackPose)
//                                .setTangent(Math.toRadians(0))
//                                .setReversed(false)
//                                .splineToLinearHeading(stackPose, Math.toRadians(180), ((pose2dDual, posePath, v) -> 30), (pose2dDual, posePath, v) -> new MinMax(-30,30))
//                };
//        Action[] oscillations = new Action[] {
//                builders[0].build(),
//                builders[1].build()
//        };
//        return new ParallelAction(
//                commandToAction(intakeSubsystem.intakeUntilPixels(false)),
//                new Action() {
//                    boolean init = false;
//                    int oscillationIndex=0;
//                    ElapsedTime time=new ElapsedTime();
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        if(!init)
//                        {
//                            init=true;
//                            time.reset();
//                        }
//
//                        boolean a = oscillations[oscillationIndex].run(telemetryPacket);
//                        if(!a){
//                            oscillations[oscillationIndex]=builders[oscillationIndex].build();
//                            oscillationIndex=(1-oscillationIndex);
//                            if(oscillationIndex==0)
//                                scheduler.schedule(intakeSubsystem.stack(0));
//                            if(oscillationIndex==1)
//                                scheduler.schedule(
//                                        new SequentialCommand(
//                                                new WaitCommand(300),
//                                                intakeSubsystem.outtake(),
//                                                new WaitCommand(300),
//                                                intakeSubsystem.intake()
//                                        ));
//                        }
//                        if(time.seconds()>7.5)
//                        {
//                            return false;
//                        }
//
//
//
//                        return !intakeSubsystem.bothPixelsDetected();
//                    }
//                }
//        );
//
//
//    }
//    public Action oscillateUntilIntake2(MecanumDrive drive, AutonomousUtils.AllianceColor color)
//    {
//        final Pose2d stackPose = AutonomousRedTrussPoses.getStack2Pose(color, AutonomousUtils.StartPosition.Truss);
//        final Pose2d nearStackPose = stackPose.plus(new Twist2d(new Vector2d(-5.75,0),0));
//
//        TrajectoryActionBuilder[] builders = new TrajectoryActionBuilder[]
//                {
//                        drive.actionBuilder(stackPose)
//                                .setTangent(stackPose.heading.log()-Math.PI)
//                                .setReversed(true)
//                                .splineToLinearHeading(nearStackPose, stackPose.heading, ((pose2dDual, posePath, v) -> 30), (pose2dDual, posePath, v) -> new MinMax(-30,30)),
//                        drive.actionBuilder(nearStackPose)
//                                .setTangent( stackPose.heading)
//                                .setReversed(false)
//                                .splineToLinearHeading(stackPose, stackPose.heading.log()-Math.PI, ((pose2dDual, posePath, v) -> 30), (pose2dDual, posePath, v) -> new MinMax(-30,30))
//                };
//        Action[] oscillations = new Action[] {
//                builders[0].build(),
//                builders[1].build()
//        };
//        return new ParallelAction(
//                commandToAction(intakeSubsystem.intakeUntilPixels(false)),
//                new Action() {
//                    boolean init = false;
//                    int oscillationIndex=0;
//                    ElapsedTime time=new ElapsedTime();
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        if(!init)
//                        {
//                            init=true;
//                            time.reset();
//                        }
//
//                        boolean a = oscillations[oscillationIndex].run(telemetryPacket);
//                        if(!a){
//                            oscillations[oscillationIndex]=builders[oscillationIndex].build();
//                            oscillationIndex=(1-oscillationIndex);
//                            if(oscillationIndex==1)
//                                scheduler.schedule(intakeSubsystem.stack(0));
//                        }
//                        if(time.seconds()>7.5)
//                        {
//                            return false;
//                        }
//
//
//
//                        return !intakeSubsystem.bothPixelsDetected();
//                    }
//                }
//        );
//
//
//    }
//    public Action stackToBackdrop(MecanumDrive drive, AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition, AutonomousUtils.AutoCase autoCase)
//    {
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//
//        return drive.actionBuilder(AutonomousRedTrussPoses.getStackPose(color, startPosition))
//                .setTangent(AutonomousUtils.mirrorColor(Math.toRadians(0),color))
//                .setReversed(false)
//                .splineToLinearHeading(cot2Pose, Math.toRadians(0))
//                .splineToLinearHeading(cotPose, Math.toRadians(0))
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(AutonomousRedTrussPoses.getBackdropPose(color,startPosition,autoCase), Math.toRadians(0))
//                .build();
//    }
//    public Action stackToBackdropExterior(MecanumDrive drive, AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition)
//    {
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//        return drive.actionBuilder(AutonomousRedTrussPoses.getStackPose(color, startPosition))
//                .setTangent(Math.toRadians(0))
//                .setReversed(false)
//                .splineToLinearHeading(cot2Pose, Math.toRadians(0))
//                .splineToLinearHeading(cotPose, Math.toRadians(0))
//                .splineToLinearHeading(AutonomousRedTrussPoses.getBackdropExterior(color,startPosition), Math.toRadians(0))
//                .build();
//    }
//    public Action backdropToStack(MecanumDrive drive, AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition, AutonomousUtils.AutoCase autoCase)
//    {
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//
//        return drive.actionBuilder(AutonomousRedTrussPoses.getBackdropPose(color,startPosition,autoCase))
//                .setTangent(Math.toRadians(180))
//
//                .setReversed(true)
//                .splineToLinearHeading(cotPose, Math.toRadians(180))
//                .splineToLinearHeading(cot2Pose, Math.toRadians(180))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(AutonomousRedTrussPoses.getStackPose(color, startPosition).plus(new Twist2d(new Vector2d(0,3),0)), Math.toRadians(180))
//                .build();
//    }
//    public Action backdropToStack2(MecanumDrive drive, AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition)
//    {
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF-1, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF-1, Math.toRadians(0)),color);
//
//        return drive.actionBuilder(AutonomousRedTrussPoses.getBackdropExterior(color,startPosition))
//                .setTangent(Math.toRadians(180))
//                .setReversed(true)
//                .splineToLinearHeading(cotPose, Math.toRadians(180))
//                .setTangent(Math.toRadians(180))
//                .splineToSplineHeading(cot2Pose, Math.toRadians(180))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(AutonomousRedTrussPoses.getStack2Pose(color, startPosition), AutonomousRedTrussPoses.getStack2Pose(color, startPosition).heading.log()-Math.PI)
//                .build();
//    }
//    public Action stack2ToBackdropExterior(MecanumDrive drive, AutonomousUtils.AllianceColor color, AutonomousUtils.StartPosition startPosition)
//    {
//        final Pose2d cotPose = AutonomousUtils.mirrorColor(new Pose2d(AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF+2, Math.toRadians(0)),color);
//        final Pose2d cot2Pose = AutonomousUtils.mirrorColor(new Pose2d(-AutonomousUtils.TILE_WIDTH, -AutonomousUtils.TILE_WIDTH_HALF, Math.toRadians(0)),color);
//        return drive.actionBuilder(AutonomousRedTrussPoses.getStack2Pose(color, startPosition))
//                .setTangent(Math.toRadians(0))
//                .setReversed(false)
//                .splineToSplineHeading(cot2Pose, Math.toRadians(0))
//                .setTangent(Math.toRadians(0))
//                .splineToSplineHeading(cotPose, Math.toRadians(0))
//                .splineToLinearHeading(AutonomousRedTrussPoses.getBackdropExterior(color,startPosition), Math.toRadians(0))
//                .build();
//    }
//}
//
