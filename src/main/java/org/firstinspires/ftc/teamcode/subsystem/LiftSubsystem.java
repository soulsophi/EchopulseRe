package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.helpers.InstantCommand;
import com.smartcluster.oracleftc.commands.helpers.ParallelCommand;
import com.smartcluster.oracleftc.commands.helpers.SequentialCommand;
import com.smartcluster.oracleftc.commands.helpers.WaitCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.Subsystem;
import com.smartcluster.oracleftc.hardware.SubsystemFlavor;
import com.smartcluster.oracleftc.math.control.MotionState;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class LiftSubsystem extends Subsystem {
    // Hardware
    private final DcMotorEx leftLift, rightLift;
    private final ServoImplEx rightHang, leftHang;
    private OracleLynxVoltageSensor voltageSensor;
    // Config
    public static PIDController leftLiftPidController = new PIDController(0.02,0,0.0006);
    public static PIDController rightLiftPidController = new PIDController(0.02,0,0.0006);
    public static TrapezoidalMotionProfile liftMotionProfile = new TrapezoidalMotionProfile(6000,8000,6000);

    public static double limitCurrentAlert = 2000;

    public static double tolerance = 30;

    public LiftSubsystem(OpMode opMode) {
        super(opMode);
        rightLift=hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift=hardwareMap.get(DcMotorEx.class, "leftLift");
        voltageSensor=hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftHang=hardwareMap.get(ServoImplEx.class, "leftHang");
        leftHang.setDirection(Servo.Direction.REVERSE);
        rightHang=hardwareMap.get(ServoImplEx.class, "rightHang");
        liftTargetPosition=0;
    }
    public static double[] lowerPositions = new double[]{0.00,0.01};
    public static double[] hangPositions = new double[] {0.63,0.64};
    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.ExpansionHubOnly;
    }
    public Command reset()
    {
        return new ParallelCommand(
                Command.builder()
                        .init(()->leftLift.setCurrentAlert(limitCurrentAlert, CurrentUnit.MILLIAMPS))
                        .update(()->leftLift.setPower(-0.5*(12.0/voltageSensor.getCachedVoltage())))
                        .finished(leftLift::isOverCurrent)
                        .end((ignored)->{
                            leftLift.setPower(0.0);
                            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        })
                        .build(),
                Command.builder()
                        .init(()->rightLift.setCurrentAlert(limitCurrentAlert, CurrentUnit.MILLIAMPS))
                        .update(()->rightLift.setPower(-0.5*(12.0/voltageSensor.getCachedVoltage())))
                        .finished(rightLift::isOverCurrent)
                        .end((ignored)->{
                            rightLift.setPower(0.0);
                            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        })
                        .build()
                ,
                new InstantCommand(()->{
                    rightHang.setPosition(lowerPositions[0]);
                    leftHang.setPosition(lowerPositions[1]);
                }));
    }
    private double initialLiftPosition=0;
    private double liftTargetPosition=0;
    public Command move(AtomicReference<Double> height)
    {
        return Command.builder()
                .init(()->{
                    initialLiftPosition=(leftLift.getCurrentPosition()+rightLift.getCurrentPosition())/2.0;
                    updateTargetHeight(height.get());
                })
                .finished(()->Math.abs(leftLift.getCurrentPosition()-liftTargetPosition)<=tolerance&&Math.abs(rightLift.getCurrentPosition()-liftTargetPosition)<=tolerance)
                .requires(this)
                .build();
    }

    public Command update()
    {
        final ElapsedTime time=new ElapsedTime();
        return Command.builder()
                .update(()->{
                    if(Math.abs(leftLift.getCurrentPosition()-liftTargetPosition)<=tolerance&&Math.abs(rightLift.getCurrentPosition()-liftTargetPosition)<=tolerance)
                    {
                        time.reset();
                        initialLiftPosition=liftTargetPosition;
                        telemetry.addData("liftSetPoint", liftTargetPosition);
                        leftLift.setPower(leftLiftPidController.update(liftTargetPosition, leftLift.getCurrentPosition())*(12.0/voltageSensor.getCachedVoltage()));
                        rightLift.setPower(rightLiftPidController.update(liftTargetPosition, rightLift.getCurrentPosition())*(12.0/voltageSensor.getCachedVoltage()));

                    }else {
                        MotionState motionState = liftMotionProfile.getMotionState(Math.abs(liftTargetPosition-initialLiftPosition), time.seconds());
                        double position = initialLiftPosition+motionState.position*Math.signum(liftTargetPosition-initialLiftPosition);
                        leftLift.setPower(leftLiftPidController.update(position, leftLift.getCurrentPosition())*(12.0/voltageSensor.getCachedVoltage()));
                        rightLift.setPower(rightLiftPidController.update(position, rightLift.getCurrentPosition())*(12.0/voltageSensor.getCachedVoltage()));
                        telemetry.addData("liftSetPoint", position);
                    }

                })
                .finished(()->Math.abs(leftLift.getCurrentPosition()-liftTargetPosition)<=tolerance&&Math.abs(rightLift.getCurrentPosition()-liftTargetPosition)<=tolerance)
                .build();

    }
    public Command hang()
    {
        return new SequentialCommand(
                new InstantCommand(()->{
                    rightHang.setPosition(hangPositions[0]);
                    leftHang.setPosition(hangPositions[1]);
                }),
                move(new AtomicReference<>(174.0))
        );
    }

    public Command lower()
    {
        return new ParallelCommand(
                new SequentialCommand(
                        new WaitCommand(500),
                        new InstantCommand(()->{
                            rightHang.setPwmDisable();
                            leftHang.setPwmDisable();

                        })
                ),
                move(new AtomicReference<>(0.0))
        );
    }
    public Command bm()
    {
        return new SequentialCommand(new ParallelCommand(
                new SequentialCommand(
                        new WaitCommand(500),
                        new InstantCommand(()->{
                            rightHang.setPwmDisable();
                            leftHang.setPwmDisable();

                        })
                ),
                move(new AtomicReference<>(0.0))
        ),

                move(new AtomicReference<>(20.0)),
                move(new AtomicReference<>(0.0)),
                move(new AtomicReference<>(20.0)),
                move(new AtomicReference<>(0.0)),
                move(new AtomicReference<>(20.0)),
                move(new AtomicReference<>(0.0)),
                move(new AtomicReference<>(20.0)),
                move(new AtomicReference<>(0.0)),
                move(new AtomicReference<>(20.0)),
                move(new AtomicReference<>(0.0))

        );
    }
    public Command unhangToIdle()
    {
        return new ParallelCommand(
                move(new AtomicReference<>(0.0)),
                new InstantCommand(()->{
                    rightHang.setPosition(lowerPositions[0]);
                    leftHang.setPosition(lowerPositions[1]);
                })
        );
    }
    public Command unhang()
    {
        return new SequentialCommand(
                move(new AtomicReference<>(176.0)),
                new InstantCommand(()->{
                    rightHang.setPwmEnable();
                    leftHang.setPwmEnable();
                })
        );
    }

    // Helpers
    public void updateTargetHeight(double height)
    {
        liftTargetPosition=heightToPosition(height);
        if(liftTargetPosition<0) liftTargetPosition=0;
        if(liftTargetPosition>3200) liftTargetPosition=3200;
    }
    public static double heightToPosition(double height)
    {
        return (height/8.0)*145.1;
    }
    public Command telemetry()
    {
        return Command.builder()
                .update(()->{
                    telemetry.addData("rightLiftPosition", rightLift.getCurrentPosition());
                    telemetry.addData("leftLiftPosition", leftLift.getCurrentPosition());
                    telemetry.addData("leftLiftPower", leftLift.getPower());
                    telemetry.addData("rightLiftPower", rightLift.getPower());
                })
                .requires(this)
                .build();
    }



    public static class Manual {
        public double liftTargetPosition;
        public double rightHangPosition = lowerPositions[0];
        public double leftHangPosition = lowerPositions[1];
    }
    public static Manual manual = new Manual();
    public Command manual()
    {
        final ElapsedTime time=new ElapsedTime();
        return Command.builder()
                .update(()->{
                    if(Math.abs(leftLift.getCurrentPosition()-manual.liftTargetPosition)<=tolerance&&Math.abs(rightLift.getCurrentPosition()-manual.liftTargetPosition)<=tolerance)
                    {
                        time.reset();
                        initialLiftPosition=manual.liftTargetPosition;
                        telemetry.addData("liftSetPoint", manual.liftTargetPosition);
                        leftLift.setPower(leftLiftPidController.update(manual.liftTargetPosition, leftLift.getCurrentPosition())*(12.0/voltageSensor.getCachedVoltage()));
                        rightLift.setPower(rightLiftPidController.update(manual.liftTargetPosition, rightLift.getCurrentPosition())*(12.0/voltageSensor.getCachedVoltage()));

                    }else {
                        MotionState motionState = liftMotionProfile.getMotionState(Math.abs(manual.liftTargetPosition-initialLiftPosition), time.seconds());
                        double position = initialLiftPosition+motionState.position*Math.signum(manual.liftTargetPosition-initialLiftPosition);
                        leftLift.setPower(leftLiftPidController.update(position, leftLift.getCurrentPosition())*(12.0/voltageSensor.getCachedVoltage()));
                        rightLift.setPower(rightLiftPidController.update(position, rightLift.getCurrentPosition())*(12.0/voltageSensor.getCachedVoltage()));
                        telemetry.addData("liftSetPoint", position);
                    }
                    telemetry.addData("leftLiftPower", leftLift.getPower());
                    telemetry.addData("rightLiftPower", rightLift.getPower());
                    leftHang.setPosition(manual.leftHangPosition);
                    rightHang.setPosition(manual.rightHangPosition);
                })
                .requires(this)
                .build();

    }



}
