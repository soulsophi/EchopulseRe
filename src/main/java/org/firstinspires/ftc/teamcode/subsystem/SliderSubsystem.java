package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.helpers.InstantCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.Subsystem;
import com.smartcluster.oracleftc.hardware.SubsystemFlavor;
import com.smartcluster.oracleftc.math.control.EdgeDetector;
import com.smartcluster.oracleftc.math.control.MotionState;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class SliderSubsystem extends Subsystem {

    // Hardware
    private final DcMotorEx slider, pitchEncoder;
    private final DigitalChannel sliderLimit;

    private final OracleLynxVoltageSensor voltageSensor;

    // Config
    public static PIDController sliderPidController = new PIDController(0.025,0,0);
    public static TrapezoidalMotionProfile sliderMotionProfile = new TrapezoidalMotionProfile(2000, 2000,1500);

    public static double tolerance=30;

    private EdgeDetector detector = new EdgeDetector(EdgeDetector.EdgeType.FALLING);
    public static int currentLimit = 4000;
    public SliderSubsystem(OpMode opMode) {
        super(opMode);
        slider=hardwareMap.get(DcMotorEx.class, "slider");
        sliderLimit=hardwareMap.get(DigitalChannel.class, "sliderLimit");
        pitchEncoder=hardwareMap.get(DcMotorEx.class, "backRight");
        voltageSensor=hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
        sliderLimit.setMode(DigitalChannel.Mode.INPUT);
        slider.setCurrentAlert(currentLimit, CurrentUnit.MILLIAMPS);
        sliderOffset=0;
    }

    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.Mixed;
    }

    public Command retract()
    {
        boolean winCondition=false;
        return Command.builder()
                .init(()->{
                    initialSliderPosition=getSliderPosition();
                    updateTargetDistance(0);
                })
                .update(()->{
                    if(slider.isOverCurrent()&&getSliderPosition()>50)
                    {
                        updateTargetDistance(100);
                    }else if(Math.abs(getSliderPosition()-targetSliderPosition)<tolerance){
                        updateTargetDistance(0);
                    }
                })
                .finished(()->Math.abs(getSliderPosition()-0)<tolerance)
                .build();
    }
    public Command reset()
    {
        return Command.builder()
                .update(()->{
                    slider.setPower(-0.8*(12.0/voltageSensor.getCachedVoltage()));
                })
                .finished(()->!sliderLimit.getState())
                .end((ignored)->{
                    slider.setPower(0.0);
                    resetPitch();
                    resetSliderPosition();
                })
                .requires(this)
                .build();

    }
    public Command move(AtomicReference<Double> distance)
    {
        return Command.builder()
                .init(()->{
                    initialSliderPosition=getSliderPosition();
                    updateTargetDistance(distance.get());
                })
                .finished(()->Math.abs(getSliderPosition()- targetSliderPosition)<=tolerance)
                .build();
    }
    public enum UpdateMode
    {
        NORMAL,
        ENSURE_CONTACT
    }
    public UpdateMode updateMode = UpdateMode.NORMAL;
    public Command closeContact()
    {
        return Command.builder().init(()->{
            updateMode=UpdateMode.ENSURE_CONTACT;
        })
                .finished(()->!sliderLimit.getState())
                .build();
    }
    public Command releaseContact()
    {
        return new InstantCommand(()->{
            updateMode=UpdateMode.NORMAL;
        });
    }
    public Command update()
    {
        final ElapsedTime time = new ElapsedTime();
        return Command.builder()
                .update(()->{
                    if(detector.calculate(sliderLimit.getState()))
                        resetSliderPosition();
                    if(updateMode==UpdateMode.NORMAL) {
                        if(Math.abs(getSliderPosition()- targetSliderPosition)<=tolerance)
                        {
                            time.reset();
                            initialSliderPosition=getSliderPosition();
                            telemetry.addData("sliderSetPoint", targetSliderPosition);
                            slider.setPower(sliderPidController.update(targetSliderPosition, getSliderPosition())*(12.0/voltageSensor.getCachedVoltage()));

                        }else {
                            MotionState motionState = sliderMotionProfile.getMotionState(Math.abs(targetSliderPosition-initialSliderPosition), time.seconds());
                            double position = initialSliderPosition+motionState.position*Math.signum(targetSliderPosition-initialSliderPosition);
                            slider.setPower(sliderPidController.update(position, getSliderPosition())*(12.0/voltageSensor.getCachedVoltage()));
                            telemetry.addData("sliderSetPoint", position);
                        }
                    }else {
                        if(sliderLimit.getState())
                            slider.setPower(-0.8);
                        else slider.setPower(-0.2);
                    }

                })
                .requires(this)
                .build();
    }

    public Command telemetry()
    {
        return Command.builder()
                .update(()->{
                    telemetry.addData("sliderPosition", getSliderPosition());
                    telemetry.addData("sliderLimit", sliderLimit.getState());
                    telemetry.addData("pitch", getSliderPitch());
                })
                .requires(this)
                .build();
    }
    // Helpers
    private double initialSliderPosition =0;
    private double targetSliderPosition =0;
    public double sliderOffset=0;
    public double pitchOffset=0;
    public void updateTargetDistance(double distance)
    {
        targetSliderPosition=distanceToPosition(distance);
    }
    public void resetPitch()
    {
        pitchOffset=(pitchEncoder.getCurrentPosition()/8192.0)*360.0;
    }
    public void resetSliderPosition()
    {
        sliderOffset=slider.getCurrentPosition()-(getSliderPitch()/360.0)*120;
    }
    public double getSliderPosition()
    {
        return slider.getCurrentPosition()-(getSliderPitch()/360.0)*120-sliderOffset;
    }
    public double getSliderPitch()
    {
        return (pitchEncoder.getCurrentPosition()/8192.0)*360.0-pitchOffset;
    }
    public static double distanceToPosition(double distance)
    {
        return (distance/120)*(24.0/25.0)*145.1;
    }
    public static double positionToDistance(double position)
    {
        return (position*120)*(25.0/24.0)/145.1;
    }


    // Manual control
    public static class Manual {
        public double targetPosition;
    }
    public static Manual manual = new Manual();
    public Command manual()
    {
        return Command.builder()
                .update(()->{
                    slider.setPower(sliderPidController.update(manual.targetPosition, slider.getCurrentPosition()));
                })
                .requires(this)
                .build();
    }
}
