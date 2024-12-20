package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.commands.helpers.InstantCommand;
import com.smartcluster.oracleftc.commands.helpers.ParallelCommand;
import com.smartcluster.oracleftc.commands.helpers.RaceCommand;
import com.smartcluster.oracleftc.commands.helpers.SequentialCommand;
import com.smartcluster.oracleftc.commands.helpers.WaitCommand;
import com.smartcluster.oracleftc.fsm.FSM;
import com.smartcluster.oracleftc.utils.input.OracleGamepad;

import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.utils.Kinematics;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;


@Config
@TeleOp(name="DuoTeleOp\uD83D\uDC6B")
public class DuoTeleOp extends LinearOpMode {
    private final CommandScheduler scheduler = new CommandScheduler();
    public enum TeleOpState {
        INTAKING_0,
        INTAKING_1,
        INTAKING_3,
        IDLE,
        LOADED,
        DEPOSITING,
        HANG,
        HANGED,
        UNHANGED,


        BM_MAXIMUS
    }
    public static double speed=50;
    public static double[][] positions = new double[][] {
            new double[] { 335, 200},
            new double[] { 360, 300},
            new double[] { 385, 400},
            new double[] { 385, 560},
    }; //Position[<Which pair of coords>][<0 = x; 1 = y>]
//    public static double[][] positions = new double[][] {
//            new double[] { 335, 200},
//            new double[] { 360, 300},
//            new double[] { 385, 400},
//            new double[] { 385, 500},
//    };
    long lastTime =-1;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime planeTime = new ElapsedTime();
        OracleGamepad driverGamepad = new OracleGamepad(gamepad1);
        OracleGamepad operatorGamepad = new OracleGamepad(gamepad2);
        MecanumSubsystem mecanumSubsystem = new MecanumSubsystem(this);
        DepositSubsystem depositSubsystem = new DepositSubsystem(this);
        SliderSubsystem sliderSubsystem = new SliderSubsystem(this);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(this);
        LiftSubsystem liftSubsystem=new LiftSubsystem(this);
        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        AtomicReference<Double> targetX=new AtomicReference<>(0.0);
        AtomicReference<Double> targetY=new AtomicReference<>(0.0);
        AtomicReference<Double> targetHeight=new AtomicReference<>(0.0);
        AtomicReference<Double> targetDistance=new AtomicReference<>(0.0);
        AtomicReference<Double> targetAngle=new AtomicReference<>(0.0);
        AtomicReference<Double> targetPitch=new AtomicReference<>(0.0);
        AtomicBoolean canAdjustPitch=new AtomicBoolean(true);
        ElapsedTime time=new ElapsedTime();
        ElapsedTime rumbleTimer = new ElapsedTime();
        IMU cringeImu=hardwareMap.get(IMU.class, "imu");
      cringeImu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        FSM<TeleOpState> intakeAndDepositFSM = FSM.<TeleOpState>builder()
                .initial(TeleOpState.IDLE)
                .transition(TeleOpState.IDLE, TeleOpState.INTAKING_0, operatorGamepad.cross.pressed(),intakeSubsystem.stack(0))
                .transition(TeleOpState.LOADED, TeleOpState.INTAKING_0, operatorGamepad.cross.pressed(),intakeSubsystem.stack(0))
                .transition(TeleOpState.INTAKING_1, TeleOpState.INTAKING_0, operatorGamepad.cross.pressed(),intakeSubsystem.stack(0))
                .transition(TeleOpState.INTAKING_3, TeleOpState.INTAKING_0, operatorGamepad.cross.pressed(),intakeSubsystem.stack(0))
                /*.transition(TeleOpState.IDLE, TeleOpState.INTAKING_1, operatorGamepad.square.pressed(),intakeSubsystem.stack(1))
                .transition(TeleOpState.LOADED, TeleOpState.INTAKING_1, operatorGamepad.square.pressed(),intakeSubsystem.stack(1))
                .transition(TeleOpState.INTAKING_0, TeleOpState.INTAKING_1, operatorGamepad.square.pressed(),intakeSubsystem.stack(1))
                .transition(TeleOpState.INTAKING_3, TeleOpState.INTAKING_1, operatorGamepad.square.pressed(),intakeSubsystem.stack(1))
                .transition(TeleOpState.IDLE, TeleOpState.INTAKING_3, operatorGamepad.triangle.pressed(),intakeSubsystem.stack(3))
                .transition(TeleOpState.LOADED, TeleOpState.INTAKING_3, operatorGamepad.triangle.pressed(),intakeSubsystem.stack(3))
                .transition(TeleOpState.INTAKING_0, TeleOpState.INTAKING_3, operatorGamepad.triangle.pressed(),intakeSubsystem.stack(3))
                .transition(TeleOpState.INTAKING_1, TeleOpState.INTAKING_3, operatorGamepad.triangle.pressed(),intakeSubsystem.stack(3))*/
                .state(TeleOpState.INTAKING_0, Command.builder()
                        .update(()->{
                            if(operatorGamepad.right_trigger.get()>0.1)
                            {
                                Command.run(intakeSubsystem.intake());
                            }else if(operatorGamepad.left_trigger.get()>0.1)
                            {
                                Command.run(intakeSubsystem.outtake());
                            }else {
                                Command.run(intakeSubsystem.stop());
                            }
                        })
                        .requires(intakeSubsystem)
                        .build())
                .state(TeleOpState.LOADED, Command.builder()
                        .update(()->{
                            if(operatorGamepad.right_trigger.get()>0.1)
                            {
                                Command.run(intakeSubsystem.intake());
                            }else if(operatorGamepad.left_trigger.get()>0.1)
                            {
                                Command.run(intakeSubsystem.outtake());
                            }else {
                                Command.run(intakeSubsystem.stop());
                            }
                        })
                        .requires(intakeSubsystem)
                        .build())
              /*  .state(TeleOpState.INTAKING_1, Command.builder()
                        .update(()->{
                            if(operatorGamepad.right_trigger.get()>0.1)
                            {
                                Command.run(intakeSubsystem.intake());
                            }else if(operatorGamepad.left_trigger.get()>0.1)
                            {
                                Command.run(intakeSubsystem.outtake());
                            }else {
                                Command.run(intakeSubsystem.stop());
                            }
                        })
                        .requires(intakeSubsystem)
                        .build())
                .state(TeleOpState.INTAKING_3, Command.builder()
                        .update(()->{
                            if(operatorGamepad.right_trigger.get()>0.1)
                            {
                                Command.run(intakeSubsystem.intake());
                            }else if(operatorGamepad.left_trigger.get()>0.1)
                            {
                                Command.run(intakeSubsystem.outtake());
                            }else {
                                Command.run(intakeSubsystem.stop());
                            }
                        })
                        .requires(intakeSubsystem)
                        .build()) */
                .state(TeleOpState.IDLE, Command.builder()
                        .update(()->{
                            if(operatorGamepad.right_trigger.get()>0.1)
                            {
                                Command.run(intakeSubsystem.intake());
                            }else if(operatorGamepad.left_trigger.get()>0.1)
                            {
                                Command.run(intakeSubsystem.outtake());
                            }else {
                                Command.run(intakeSubsystem.stop());
                            }
                        })
                        .requires(intakeSubsystem)
                        .build())
                .transition(TeleOpState.INTAKING_0, TeleOpState.LOADED,operatorGamepad.dpad_right.pressed(),
                        new SequentialCommand(
                                new InstantCommand(()->{
                                    gamepad1.rumble(1,1, 500);
                                    gamepad2.rumble(1,1, 500);
                                }),
                                new WaitCommand(350),
                                intakeSubsystem.intake(),
                                new WaitCommand(250),
                                intakeSubsystem.stop(),
                                new ParallelCommand(
                                        intakeSubsystem.lift(),
                                        sliderSubsystem.closeContact()
                                ),
                                new WaitCommand(100),
                                intakeSubsystem.outtake(),
                                new WaitCommand(1000),
                                sliderSubsystem.releaseContact(),
                                intakeSubsystem.stop()
                        ))
               /* .transition(TeleOpState.INTAKING_1, TeleOpState.LOADED,operatorGamepad.dpad_right.pressed(),new SequentialCommand(
                        new InstantCommand(()->{
                            gamepad1.rumble(1,1, 500);
                            gamepad2.rumble(1,1, 500);
                        }),
                        new WaitCommand(350),
                        intakeSubsystem.intake(),
                        new WaitCommand(250),
                        intakeSubsystem.stop(),
                        new ParallelCommand(
                                intakeSubsystem.lift(),
                                sliderSubsystem.closeContact()
                        ),
                        new WaitCommand(100),
                        intakeSubsystem.outtake(),
                        new WaitCommand(1000),
                        intakeSubsystem.stop()
                ))
                .transition(TeleOpState.INTAKING_3, TeleOpState.LOADED,operatorGamepad.dpad_right.pressed(),new SequentialCommand(
                        new InstantCommand(()->{
                            gamepad1.rumble(1,1, 500);
                            gamepad2.rumble(1,1, 500);
                        }),
                        new WaitCommand(350),
                        intakeSubsystem.intake(),
                        new WaitCommand(250),
                        intakeSubsystem.stop(),
                        new ParallelCommand(
                                intakeSubsystem.lift(),
                                sliderSubsystem.closeContact()
                        ),
                        new WaitCommand(100),
                        intakeSubsystem.outtake(),
                        new WaitCommand(1000),
                        intakeSubsystem.stop()
                ))*/

                .transition(TeleOpState.LOADED, TeleOpState.DEPOSITING, operatorGamepad.dpad_up.pressed(), new SequentialCommand(
                        new InstantCommand(()->{
                            double[] ik = Kinematics.inverseKinematics(positions[3][0], positions[3][1]);
                            targetX.set(positions[3][0]);
                            targetY.set(positions[3][1]);
                            targetDistance.set(ik[0]);
                            targetHeight.set(ik[1]);
                            targetAngle.set(ik[2]);
                            targetPitch.set(ik[3]);
                        }),
                        new SequentialCommand(
                                depositSubsystem.pitch(targetAngle),
                                new ParallelCommand(
                                sliderSubsystem.move(targetDistance),
                                liftSubsystem.move(targetHeight),
                                intakeSubsystem.idleDepositing()
                        )
                )))
                .transition(TeleOpState.IDLE, TeleOpState.DEPOSITING, operatorGamepad.dpad_down.pressed(), new SequentialCommand(
                        new InstantCommand(()->{
                            double[] ik = Kinematics.inverseKinematics(positions[0][0], positions[0][1]);
                            targetX.set(positions[0][0]);
                            targetY.set(positions[0][1]);
                            targetDistance.set(ik[0]);
                            targetHeight.set(ik[1]);
                            targetAngle.set(ik[2]);
                            targetPitch.set(ik[3]);
                        }),
                        new ParallelCommand(
                                sliderSubsystem.move(targetDistance),
                                liftSubsystem.move(targetHeight),
                                depositSubsystem.pitch(targetAngle),
                                intakeSubsystem.idleDepositing()
                        )
                ))
                .transition(TeleOpState.IDLE, TeleOpState.DEPOSITING, operatorGamepad.dpad_left.pressed(), new SequentialCommand(
                        new InstantCommand(()->{
                            double[] ik = Kinematics.inverseKinematics(positions[1][0], positions[1][1]);
                            targetX.set(positions[1][0]);
                            targetY.set(positions[1][1]);
                            targetDistance.set(ik[0]);
                            targetHeight.set(ik[1]);
                            targetAngle.set(ik[2]);
                            targetPitch.set(ik[3]);
                        }),
                        new ParallelCommand(
                                sliderSubsystem.move(targetDistance),
                                liftSubsystem.move(targetHeight),
                                depositSubsystem.pitch(targetAngle),
                                intakeSubsystem.idleDepositing()
                        )
                ))
                .transition(TeleOpState.IDLE, TeleOpState.DEPOSITING, operatorGamepad.dpad_right.pressed(), new SequentialCommand(
                        new InstantCommand(()->{
                            double[] ik = Kinematics.inverseKinematics(positions[2][0], positions[2][1]);
                            targetX.set(positions[2][0]);
                            targetY.set(positions[2][1]);
                            targetDistance.set(ik[0]);
                            targetHeight.set(ik[1]);
                            targetAngle.set(ik[2]);
                            targetPitch.set(ik[3]);
                        }),
                        new SequentialCommand(
                                sliderSubsystem.move(targetDistance),
                                liftSubsystem.move(targetHeight),
                                depositSubsystem.pitch(targetAngle),
                                intakeSubsystem.idleDepositing()
                        )
                ))
                .transition(TeleOpState.IDLE, TeleOpState.DEPOSITING, operatorGamepad.dpad_up.pressed(), new SequentialCommand(
                        new InstantCommand(()->{
                            double[] ik = Kinematics.inverseKinematics(positions[3][0], positions[3][1]);
                            targetX.set(positions[3][0]);
                            targetY.set(positions[3][1]);
                            targetDistance.set(ik[0]);
                            targetHeight.set(ik[1]);
                            targetAngle.set(ik[2]);
                            targetPitch.set(ik[3]);
                        }),
                        new ParallelCommand(
                                depositSubsystem.pitch(targetAngle),
                                sliderSubsystem.move(targetDistance),
                                liftSubsystem.move(targetHeight),
                                intakeSubsystem.idleDepositing()
                        )
                ))

                .state(TeleOpState.DEPOSITING, new ParallelCommand(
                        Command.builder()
                                .update(()->{
                                    if(driverGamepad.cross.pressed().get())
                                    {
                                        Command.run(depositSubsystem.open());
                                    }

                                }).build(),
                        new SequentialCommand(
                        new WaitCommand(500),
                        Command.builder()
                        .update(()->{

                            if(operatorGamepad.right_bumper.pressed().get())
                            {
                                double newX = targetX.get()+Math.cos(Math.toRadians(60))*70;
                                double newY = targetY.get()+Math.cos(Math.toRadians(60))*70;
                                targetX.set(newX);
                                targetY.set(newY);
                                double[] ik = Kinematics.inverseKinematics(targetX.get(), targetY.get());
                                targetDistance.set(ik[0]);
                                targetHeight.set(ik[1]);
                                targetAngle.set(ik[2]);
                                targetPitch.set(ik[3]);
                                sliderSubsystem.updateTargetDistance(targetDistance.get());
                                liftSubsystem.updateTargetHeight(targetHeight.get());
                                depositSubsystem.updateTargetAngle(targetAngle.get());
                                canAdjustPitch.set(false);
                                scheduler.schedule(new SequentialCommand(
                                        new WaitCommand(800),
                                        new InstantCommand(()->canAdjustPitch.set(true))
                                ));
                            }
                            if(operatorGamepad.left_bumper.pressed().get())
                            {
                                double newX = targetX.get()-Math.cos(Math.toRadians(60))*70;
                                double newY = targetY.get()-Math.cos(Math.toRadians(60))*70;
                                targetX.set(newX);
                                targetY.set(newY);
                                double[] ik = Kinematics.inverseKinematics(targetX.get(), targetY.get());
                                targetDistance.set(ik[0]);
                                targetHeight.set(ik[1]);
                                targetAngle.set(ik[2]);
                                targetPitch.set(ik[3]);
                                sliderSubsystem.updateTargetDistance(targetDistance.get());
                                liftSubsystem.updateTargetHeight(targetHeight.get());
                                depositSubsystem.updateTargetAngle(targetAngle.get());
                                canAdjustPitch.set(false);
                                scheduler.schedule(new SequentialCommand(
                                        new WaitCommand(800),
                                        new InstantCommand(()->canAdjustPitch.set(true))
                                ));
                            }
                            if(canAdjustPitch.get()&&sliderSubsystem.getSliderPitch()-targetPitch.get()>3.0)
                            {
                                double[] k = Kinematics.forwardKinematics(targetDistance.get(), sliderSubsystem.getSliderPitch());
                                targetX.set(k[0]);
                                targetY.set(k[1]);
                                double[] ik = Kinematics.inverseKinematics(targetX.get(), targetY.get());
                                targetDistance.set(ik[0]);
                                targetHeight.set(ik[1]);
                                targetAngle.set(ik[2]);
                                targetPitch.set(ik[3]);
                                sliderSubsystem.updateTargetDistance(targetDistance.get());
                                liftSubsystem.updateTargetHeight(targetHeight.get());
                                depositSubsystem.updateTargetAngle(targetAngle.get());
                            }
                        })
                        .build())))
                .transition(TeleOpState.DEPOSITING, TeleOpState.IDLE, driverGamepad.dpad_down.pressed(),
                        new SequentialCommand(
                                new WaitCommand(50),
                                new SequentialCommand(
                                    new InstantCommand(()->{
                                        double[] ik = Kinematics.inverseKinematics(positions[1][0], positions[1][1]);
                                        targetX.set(positions[1][0]);
                                        targetY.set(positions[1][1]);
                                        targetDistance.set(ik[0]);
                                        targetHeight.set(ik[1]);
                                        targetAngle.set(ik[2]);
                                        targetPitch.set(ik[3]);
                                    }),
                                    new WaitCommand(100),
                                    new InstantCommand(()->{
                                        double newX = targetX.get()+Math.cos(Math.toRadians(60))*70;
                                        double newY = targetY.get()+Math.cos(Math.toRadians(60))*70;
                                        targetX.set(newX);
                                        targetY.set(newY);
                                        double[] ik = Kinematics.inverseKinematics(targetX.get(), targetY.get());
                                        targetDistance.set(ik[0]);
                                        targetHeight.set(ik[1]);
                                        targetAngle.set(ik[2]);
                                        targetPitch.set(ik[3]);
                                    })
                                ),
                                new ParallelCommand(
                                        sliderSubsystem.move(targetDistance),
                                        liftSubsystem.move(targetHeight),
                                        depositSubsystem.pitch(targetAngle)
                                        ),
                                new WaitCommand(400),
                                new ParallelCommand(
                                        new SequentialCommand(
                                                new WaitCommand(500),
                                                sliderSubsystem.retract()
                                        ),
                                        new SequentialCommand(
                                                new WaitCommand(250),
                                                depositSubsystem.pitch(new AtomicReference<>(13.86)),
                                                depositSubsystem.close()
                                                ),
                                        new SequentialCommand(
                                                new WaitCommand(750),
                                                liftSubsystem.move(new AtomicReference<>(0.0))
                                                )
                                ),
                                intakeSubsystem.lift()))
                .transition(TeleOpState.IDLE, TeleOpState.DEPOSITING, operatorGamepad.circle.pressed(), new SequentialCommand(
                        new InstantCommand(()->{
                            double[] ik = Kinematics.inverseKinematics(targetX.get(), targetY.get());
                            targetDistance.set(ik[0]);
                            targetHeight.set(ik[1]);
                            targetAngle.set(ik[2]);
                            targetPitch.set(ik[3]);
                        }),
                        new ParallelCommand(
                                sliderSubsystem.move(targetDistance),
                                liftSubsystem.move(targetHeight),
                                depositSubsystem.pitch(targetAngle),
                                intakeSubsystem.idleDepositing()
                        )
                ))
                .transition(TeleOpState.DEPOSITING, TeleOpState.DEPOSITING, operatorGamepad.dpad_down.pressed(), new SequentialCommand(
                        new InstantCommand(()->{
                            double[] ik = Kinematics.inverseKinematics(positions[0][0], positions[0][1]);
                            targetX.set(positions[0][0]);
                            targetY.set(positions[0][1]);

                            targetDistance.set(ik[0]);
                            targetHeight.set(ik[1]);
                            targetAngle.set(ik[2]);
                            targetPitch.set(ik[3]);
                        }),
                        new ParallelCommand(
                                sliderSubsystem.move(targetDistance),
                                liftSubsystem.move(targetHeight),
                                depositSubsystem.pitch(targetAngle)
                        )
                ))
                .transition(TeleOpState.DEPOSITING, TeleOpState.DEPOSITING, operatorGamepad.dpad_left.pressed(), new SequentialCommand(
                        new InstantCommand(()->{
                            double[] ik = Kinematics.inverseKinematics(positions[1][0], positions[1][1]);
                            targetX.set(positions[1][0]);
                            targetY.set(positions[1][1]);
                            targetDistance.set(ik[0]);
                            targetHeight.set(ik[1]);
                            targetAngle.set(ik[2]);
                            targetPitch.set(ik[3]);
                        }),
                        new ParallelCommand(
                                sliderSubsystem.move(targetDistance),
                                liftSubsystem.move(targetHeight),
                                depositSubsystem.pitch(targetAngle)
                        )
                ))
                .transition(TeleOpState.DEPOSITING, TeleOpState.DEPOSITING, operatorGamepad.dpad_right.pressed(), new SequentialCommand(
                        new InstantCommand(()->{
                            double[] ik = Kinematics.inverseKinematics(positions[2][0], positions[2][1]);
                            targetX.set(positions[2][0]);
                            targetY.set(positions[2][1]);
                            targetDistance.set(ik[0]);
                            targetHeight.set(ik[1]);
                            targetAngle.set(ik[2]);
                            targetPitch.set(ik[3]);
                        }),
                        new ParallelCommand(
                                sliderSubsystem.move(targetDistance),
                                liftSubsystem.move(targetHeight),
                                depositSubsystem.pitch(targetAngle)
                        )
                ))
                .transition(TeleOpState.DEPOSITING, TeleOpState.DEPOSITING, operatorGamepad.dpad_up.pressed(), new SequentialCommand(
                        new InstantCommand(()->{
                            double[] ik = Kinematics.inverseKinematics(positions[3][0], positions[3][1]);
                            targetX.set(positions[3][0]);
                            targetY.set(positions[3][1]);
                            targetDistance.set(ik[0]);
                            targetHeight.set(ik[1]);
                            targetAngle.set(ik[2]);
                            targetPitch.set(ik[3]);
                        }),
                        new ParallelCommand(
                                sliderSubsystem.move(targetDistance),
                                liftSubsystem.move(targetHeight),
                                depositSubsystem.pitch(targetAngle)
                        )
                ))


                /*.transition(TeleOpState.IDLE, TeleOpState.HANG, driverGamepad.dpad_up.pressed(),
                        new SequentialCommand(
                                intakeSubsystem.idle(),
                                liftSubsystem.hang()))
                .transition(TeleOpState.HANG, TeleOpState.HANGED,driverGamepad.dpad_down.pressed(), liftSubsystem.lower())
                .transition(TeleOpState.HANGED, TeleOpState.UNHANGED, driverGamepad.dpad_up.pressed(), liftSubsystem.unhang())
                .transition(TeleOpState.UNHANGED, TeleOpState.HANGED, driverGamepad.dpad_up.pressed(), new InstantCommand(()->{}))
                .transition(TeleOpState.HANG, TeleOpState.BM_MAXIMUS, driverGamepad.cross.pressed(),new ParallelCommand(liftSubsystem.bm(), sliderSubsystem.move(new AtomicReference<>(300.0))))
                .transition(TeleOpState.UNHANGED, TeleOpState.IDLE, driverGamepad.dpad_down.pressed(),
                        new SequentialCommand(
                                liftSubsystem.unhangToIdle(),
                                intakeSubsystem.lift()
                        ))*/

                .build();

        waitForStart();
        planeTime.reset();
        rumbleTimer.reset();
        scheduler.schedule(
                new SequentialCommand(
                        new WaitCommand(110000),
                        new InstantCommand(()->{
                            gamepad1.rumble(1.0,1.0,1000);
                            gamepad2.rumble(1.0,1.0,1000);
                        })
                )
        );
        scheduler.schedule(new SequentialCommand(
                depositSubsystem.reset(),
                new ParallelCommand(
                        new SequentialCommand(
                                new WaitCommand(1000),
                                liftSubsystem.reset()
                        ),
                        sliderSubsystem.reset()
                ),
                intakeSubsystem.reset(),
                new RaceCommand(
                        depositSubsystem.update(),
                        depositSubsystem.pitch(new AtomicReference<>(13.86))
                ),
                new InstantCommand(sliderSubsystem::resetPitch),
                new ParallelCommand(
                        sliderSubsystem.update(),
                        depositSubsystem.update(),
                        liftSubsystem.update()
                )
        ));
        for (LynxModule module : modules)
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        scheduler.schedule(mecanumSubsystem.drive(driverGamepad));
        scheduler.schedule(intakeSubsystem.telemetry());
        scheduler.schedule(sliderSubsystem.telemetry());
        scheduler.schedule(liftSubsystem.telemetry());
        ElapsedTime performanceTime = new ElapsedTime();
        while(opModeIsActive()&&!isStopRequested())
        {
            driverGamepad.process();
            operatorGamepad.process();
            for (LynxModule module : modules)
                module.clearBulkCache();

            telemetry.addData("state", intakeAndDepositFSM.getCurrentState());
            if(intakeAndDepositFSM.getCurrentTransition()!=null)
            {
                telemetry.addData("targetState", intakeAndDepositFSM.getCurrentTransition().getTargetState());
            }else telemetry.addData("targetState", new Object());

            performanceTime.reset();
            scheduler.update();
            telemetry.addData("schedulerTime(ms)",performanceTime.milliseconds());

            performanceTime.reset();
            intakeAndDepositFSM.update();
            telemetry.addData("fsmTime(ms)",performanceTime.milliseconds());

            telemetry.addData("targetAngle", targetAngle.get());
            telemetry.addData("targetDistance", targetDistance.get());
            telemetry.addData("targetHeight", targetHeight.get());
            telemetry.addData("targetPitch", targetPitch.get());
            long currentTime = System.nanoTime();
            if(lastTime!=-1)
                telemetry.addData("loop(hz)", 1000/((currentTime-lastTime)/1E6));
            lastTime=System.nanoTime();
            telemetry.update();



        }
    }
}
