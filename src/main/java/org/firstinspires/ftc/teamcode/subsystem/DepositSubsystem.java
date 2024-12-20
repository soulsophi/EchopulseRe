package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.helpers.InstantCommand;
import com.smartcluster.oracleftc.commands.helpers.SequentialCommand;
import com.smartcluster.oracleftc.commands.helpers.WaitCommand;
import com.smartcluster.oracleftc.hardware.Subsystem;
import com.smartcluster.oracleftc.hardware.SubsystemFlavor;
import com.smartcluster.oracleftc.math.control.MotionState;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class DepositSubsystem extends Subsystem {
    // Hardware

    private final ServoImplEx rightDeposit, leftDeposit, pitchDeposit;

    // Config
    public static TrapezoidalMotionProfile depositMotionProfile = new TrapezoidalMotionProfile(3,4,4);
    public static double[] closedPositions = new double[] {0.32, 0.34};
    public static double[] openPositions = new double[] {-0.33, -0.31};
    public DepositSubsystem(OpMode opMode) {
        super(opMode);

        rightDeposit=hardwareMap.get(ServoImplEx.class, "rightDeposit");
        leftDeposit=hardwareMap.get(ServoImplEx.class, "leftDeposit");
        pitchDeposit=hardwareMap.get(ServoImplEx.class, "pitchDeposit");
        closed[0]=false;
        closed[1]=false;
    }

    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.Mixed;
    }
    public Command reset()
    {
        return new SequentialCommand(
                new InstantCommand(()->{
                    pitchDeposit.setPosition(angleToPosition(60));
                    updateTargetAngle(60);
                }),
                new WaitCommand(500)
        );
    }


    public boolean[] closed = new boolean[2];
    public Command close()
    {
        return new InstantCommand(()->{
            rightDeposit.setPosition(closedPositions[0]);
            leftDeposit.setPosition(closedPositions[1]);
            closed[0]=closed[1]=true;
        });
    }

    public Command open()
    {
        return new InstantCommand(()->{
            rightDeposit.setPosition(openPositions[0]);
            leftDeposit.setPosition(openPositions[1]);
            closed[0]=closed[1]=false;
        });
    }
    public Command pitch(AtomicReference<Double> angle)
    {
        return Command.builder()
                .init(()->{
                    initialPitchPosition=pitchDeposit.getPosition();
                    targetPitchPosition=angleToPosition(angle.get());
                })
                .finished(()->Math.abs(pitchDeposit.getPosition()- targetPitchPosition)<=0.05)
                .requires(this)
                .build();
    }
    public Command update()
    {
        final ElapsedTime time = new ElapsedTime();
        return Command.builder()
                .update(()->{
                    if(Math.abs(pitchDeposit.getPosition()- targetPitchPosition)<=0.05)
                    {
                        time.reset();
                        initialPitchPosition=targetPitchPosition;
                        telemetry.addData("pitchSetPoint", targetPitchPosition);
                        pitchDeposit.setPosition(targetPitchPosition);

                    }else {
                        MotionState motionState = depositMotionProfile.getMotionState(Math.abs(targetPitchPosition-initialPitchPosition), time.seconds());
                        double position = initialPitchPosition+motionState.position*Math.signum(targetPitchPosition-initialPitchPosition);
                        pitchDeposit.setPosition(position);
                        telemetry.addData("pitchSetPoint", position);
                        telemetry.addData("pitchPosition", pitchDeposit.getPosition());

                    }
                })
                .requires(this)
                .build();
    }


    // Helpers
    public boolean SampleDropped()
    {
        return (leftDeposit.getPosition()>0.1) && (rightDeposit.getPosition()>0.1);
    }

    private double initialPitchPosition=0;
    private double targetPitchPosition=0;
    public void updateTargetAngle(double angle)
    {
        if(!Double.isNaN(angle))
            targetPitchPosition=angleToPosition(angle);
    }
    public static double angleToPosition(double angle)
    {
        // 0.49 -- 90
        //
        return ((178.26+(angle-90))-19.5)/324.0;
    }

    public static class Manual {
        public double rightDepositPosition = closedPositions[0];
        public double leftDepositPosition = closedPositions[1];
        public double pitchDepositPosition=0.49;
    }

    public static Manual manual = new Manual();

    public Command manual()
    {
        return Command.builder()
                .update(()->{
                    rightDeposit.setPosition(manual.rightDepositPosition);
                    leftDeposit.setPosition(manual.leftDepositPosition);
                    pitchDeposit.setPosition(manual.pitchDepositPosition);
                })
                .requires(this)
                .build();
    }

}
