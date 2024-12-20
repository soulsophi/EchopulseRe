//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.smartcluster.oracleftc.commands.CommandScheduler;
//import com.smartcluster.oracleftc.commands.helpers.ConditionalCommand;
//import com.smartcluster.oracleftc.commands.helpers.InstantCommand;
//import com.smartcluster.oracleftc.commands.helpers.ParallelCommand;
//import com.smartcluster.oracleftc.commands.helpers.SequentialCommand;
//import com.smartcluster.oracleftc.commands.helpers.ToggleCommand;
//import com.smartcluster.oracleftc.utils.UseConfig;
//import com.smartcluster.oracleftc.utils.input.Binding;
//import com.smartcluster.oracleftc.utils.input.GamePad;
//
//import org.firstinspires.ftc.teamcode.R;
//import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystemOld;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystemOld;
//import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.MecanumSubsystem;
//import org.firstinspires.ftc.teamcode.utils.Kinematics;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
//
//@SuppressWarnings("unused")
//@TeleOp(name="SoloTeleOp\uD83D\uDC64")
//@UseConfig(R.xml.config)
//public class SoloTeleOp extends LinearOpMode {
//    private final CommandScheduler scheduler = new CommandScheduler();
//    public enum TeleOpState {
//      INTAKING,
//      IDLE,
//      DEPOSITING
//    };
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        GamePad gamepad = new GamePad(gamepad1);
//        MecanumSubsystem mecanumSubsystem = new MecanumSubsystem(this);
//        IntakeSubsystemOld intakeSubsystem = new IntakeSubsystemOld(this);
//        DepositSubsystemOld depositSubsystem = new DepositSubsystemOld(this);
//        LiftSubsystem liftSubsystem = new LiftSubsystem(this);
//        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
//
//        AtomicReference<Double> targetY= new AtomicReference<>((double) 0);
//        AtomicReference<Double> targetX=new AtomicReference<>((double) 0);
//        AtomicReference<Double> targetHeight= new AtomicReference<>((double) 0);
//        AtomicReference<Double> targetDistance=new AtomicReference<>((double) 0);
//        AtomicReference<Double> targetAngle=new AtomicReference<>((double) 0);
//        AtomicReference<TeleOpState> state= new AtomicReference<>(TeleOpState.IDLE);
//
//        waitForStart();
//        for (LynxModule module: modules)
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//
//        scheduler.schedule(mecanumSubsystem.drive(gamepad));
//        scheduler.schedule(intakeSubsystem.intake(gamepad));
//        scheduler.schedule(
//                new SequentialCommand(
//                        liftSubsystem.zero(),
//                        liftSubsystem.update()
//                ));
//        scheduler.schedule(
//                new SequentialCommand(
//                        depositSubsystem.zero(),
//                        depositSubsystem.update()
//                ));
//
//        scheduler.bind(gamepad.cross.pressed(), new ConditionalCommand(
//                ()->intakeSubsystem.state== IntakeSubsystemOld.IntakeState.LIFTED,
//                intakeSubsystem.drop(),
//                intakeSubsystem.lift()
//        ));
//        scheduler.bind(gamepad.circle.pressed(), new ConditionalCommand(
//                ()->state.get()==TeleOpState.IDLE,
//                new SequentialCommand(
//                        intakeSubsystem.idle(),
//                        new ParallelCommand(
//                                depositSubsystem.move(targetDistance),
//                                liftSubsystem.move(targetHeight),
//                                depositSubsystem.pitch(targetAngle)
//                        ),
//                        new InstantCommand(()-> state.set(TeleOpState.DEPOSITING))
//                ),
//                new SequentialCommand(
//                        depositSubsystem.pitch(new AtomicReference<>((double)0)),
//                        new ParallelCommand(
//                            depositSubsystem.move(new AtomicReference<>((double) 0)),
//                            liftSubsystem.move(new AtomicReference<>((double) 0)),
//                            new InstantCommand(()-> state.set(TeleOpState.IDLE))
//                        ),
//                        intakeSubsystem.lift()
//                )
//        ));
//        Binding<Boolean> dpad_up = gamepad.dpad_up.pressed();
//        Binding<Boolean> dpad_left = gamepad.dpad_left.pressed();
//        Binding<Boolean> dpad_down = gamepad.dpad_down.pressed();
//        Binding<Boolean> dpad_right = gamepad.dpad_right.pressed();
//        scheduler.bind(gamepad.square.pressed(), new ToggleCommand(
//                depositSubsystem.open(),
//                depositSubsystem.close()
//        ));
//        while(opModeIsActive())
//        {
//            for (LynxModule module: modules)
//                module.clearBulkCache();
//
//
//
//            if(dpad_up.processAndGet()){
//                targetY.updateAndGet(v->v+50);
//                double[] newParameters = Kinematics.inverseKinematics(targetX.get(), targetY.get());
//                targetDistance.set(newParameters[0]);
//                targetHeight.set(newParameters[1]);
//                targetAngle.set(newParameters[2]);
//                if(state.get()==TeleOpState.DEPOSITING)
//                {
//                    scheduler.schedule(new ParallelCommand(
//                            depositSubsystem.move(targetDistance),
//                            liftSubsystem.move(targetHeight),
//                            depositSubsystem.pitch(targetAngle))
//                    );
//                }
//            }
//            if(dpad_down.processAndGet()){
//                targetY.updateAndGet(v->Math.max(0,v-50));
//                double[] newParameters = Kinematics.inverseKinematics(targetX.get(), targetY.get());
//                targetDistance.set(newParameters[0]);
//                targetHeight.set(newParameters[1]);
//                targetAngle.set(newParameters[2]);
//                if(state.get()==TeleOpState.DEPOSITING)
//                {
//                    scheduler.schedule(
//                            new ParallelCommand(
//                                    depositSubsystem.move(targetDistance),
//                                    liftSubsystem.move(targetHeight),
//                                    depositSubsystem.pitch(targetAngle)
//                            )
//                    );
//                }
//            }
//            if(dpad_right.processAndGet()){
//                targetX.updateAndGet(v->v+50);
//                double[] newParameters = Kinematics.inverseKinematics(targetX.get(), targetY.get());
//                targetDistance.set(newParameters[0]);
//                targetHeight.set(newParameters[1]);
//                targetAngle.set(newParameters[2]);
//                if(state.get()==TeleOpState.DEPOSITING)
//                {
//                    scheduler.schedule(new ParallelCommand(
//                            depositSubsystem.move(targetDistance),
//                            liftSubsystem.move(targetHeight),
//                            depositSubsystem.pitch(targetAngle)));
//                }
//            };
//            if(dpad_left.processAndGet()) {
//                targetX.updateAndGet(v->Math.max(0,v-50));
//                double[] newParameters = Kinematics.inverseKinematics(targetX.get(), targetY.get());
//                targetDistance.set(newParameters[0]);
//                targetHeight.set(newParameters[1]);
//                targetAngle.set(newParameters[2]);
//                if(state.get()==TeleOpState.DEPOSITING)
//                {
//                    scheduler.schedule(new ParallelCommand(
//                            depositSubsystem.move(targetDistance),
//                            liftSubsystem.move(targetHeight),
//                            depositSubsystem.pitch(targetAngle)));
//                }
//
//            }
//            telemetry.update();
//            scheduler.updateBindings();
//            scheduler.update();
//        }
//    }
//
//}
