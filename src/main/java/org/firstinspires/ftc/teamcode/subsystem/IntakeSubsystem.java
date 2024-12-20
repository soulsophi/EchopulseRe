package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.meepmeep.AutonomousUtils;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.helpers.ConditionalCommand;
import com.smartcluster.oracleftc.commands.helpers.InstantCommand;
import com.smartcluster.oracleftc.commands.helpers.SequentialCommand;
import com.smartcluster.oracleftc.commands.helpers.WaitCommand;
import com.smartcluster.oracleftc.hardware.Subsystem;
import com.smartcluster.oracleftc.hardware.SubsystemFlavor;
import com.smartcluster.oracleftc.math.control.MotionState;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import java.util.HashSet;
import java.util.Set;

@Config
public class IntakeSubsystem extends Subsystem {

    // Hardware
    private final ServoImplEx rightV4B, leftV4B, pitchV4B, leftIntake, rightIntake;
    private final DcMotorEx intake;

    // Config
    public static double servoOffset=-0.044;
    public static double[][] stackPositions = new double[][]{
            new double[] {0.03, 0.90-0.38},
            new double[] {0.03, 0.88-0.38},
            new double[] {0.03, 0.86-0.38},
            new double[] {0.27, 0.82},
            new double[] {0.03, 0.82-0.38},
    };
//    public static class Manual {
//        public double intakePosition=liftedPosition[0];
//        public double pitchPosition=liftedPosition[1];
//        public double servoOffset=IntakeSubsystem.servoOffset;
//        public double leftIntakePosition=grabberPositions[0][1];
//        public double rightIntakePosition=grabberPositions[0][0];
//        public double intakePower=0.0;
//    };

    public static double[][] grabberPositions = new double[][] {
            new double[] {0.5,0.295},
            new double[] {1, 0.95}
    };
    public static double[] idlePosition = new double[] {0.25, 0.84-0.38};
    public static double[] liftedPosition=new double[] {0.5,0.295};

    public static TrapezoidalMotionProfile v4bMotionProfile = new TrapezoidalMotionProfile(3.5,4,3);
    public static TrapezoidalMotionProfile pitchV4BMotionProfile = new TrapezoidalMotionProfile(3,5,3);

    public IntakeSubsystem(OpMode opMode) {
        super(opMode);
        rightV4B=hardwareMap.get(ServoImplEx.class, "rightV4B");
        leftV4B=hardwareMap.get(ServoImplEx.class, "leftV4B");
        pitchV4B=hardwareMap.get(ServoImplEx.class, "pitchV4B");
        leftIntake=hardwareMap.get(ServoImplEx.class, "leftIntake");
        rightIntake=hardwareMap.get(ServoImplEx.class, "rightIntake");
        intake=hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftV4B.setPwmRange(new PwmControl.PwmRange(550,2450));
        rightV4B.setPwmRange(new PwmControl.PwmRange(550,2450));
//        Command.run(reset());
    }

    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.ControlHubOnly;
    }

    public Command reset()
    {
        return new InstantCommand(()->{
           leftV4B.setPosition(liftedPosition[0]+servoOffset);
           rightV4B.setPosition(liftedPosition[0]);
           pitchV4B.setPosition(liftedPosition[1]);
           rightIntake.setPosition(grabberPositions[0][0]);
           leftIntake.setPosition(grabberPositions[0][1]);
        });
    }

    public Command intake()
    {
        return new InstantCommand(()->intake.setPower(-1.0));
    }

    public Command lockColor(AutonomousUtils.AllianceColor color)
    {
        return new InstantCommand(()->{
            if(color== AutonomousUtils.AllianceColor.Red)rightIntake.setPosition(grabberPositions[1][0]);
            else leftIntake.setPosition(grabberPositions[1][1]);
        });
    }
    public Command outtake()
    {
        return new InstantCommand(()->intake.setPower(0.6));
    }
    public Command stop()
    {
        return new InstantCommand(()->intake.setPower(0.0));
    }


    public Command stack(int stack)
    {
        if(stack<0) stack=0;
        if(stack>4) stack=4;
        return move(stackPositions[stack][0],stackPositions[stack][1]);
    }
    public Command lift()
    {
        return new IntakeMoveCommand(liftedPosition[0], liftedPosition[1]);
    }
    public Command move(double v4bPosition, double pitchPosition)
    {
        return new IntakeMoveCommand(v4bPosition, pitchPosition);
    }
    public Command idle()
    {
        return new IntakeMoveCommand(idlePosition[0], idlePosition[1]);
    }
    public Command idleDepositing()
    {
        return new IntakeMoveCommand(0.25, 0.24);
    }
    // Helpers

    public class IntakeMoveCommand extends Command {
        private final double v4bTargetPosition, pitchV4BTargetPosition;
        private final ElapsedTime time = new ElapsedTime();
        public IntakeMoveCommand(double v4bTargetPosition, double pitchV4BTargetPosition)
        {
            this.v4bTargetPosition=v4bTargetPosition;
            this.pitchV4BTargetPosition=pitchV4BTargetPosition;
        }

        private double initialV4BPosition, initialPitchV4BPosition;

        @Override
        public void init() {
            initialV4BPosition=rightV4B.getPosition();
            initialPitchV4BPosition=pitchV4B.getPosition();
            time.reset();
        }

        @Override
        public void update() {
            double v4bDistance = v4bTargetPosition-initialV4BPosition;
            double pitchV4BDistance = pitchV4BTargetPosition-initialPitchV4BPosition;

            MotionState v4bMotionState = v4bMotionProfile.getMotionState(Math.abs(v4bDistance), time.seconds());
            MotionState pitchV4bMotionState = pitchV4BMotionProfile.getMotionState(Math.abs(pitchV4BDistance),time.seconds());

            rightV4B.setPosition(initialV4BPosition+v4bMotionState.position*Math.signum(v4bDistance));
            leftV4B.setPosition(initialV4BPosition+servoOffset+v4bMotionState.position*Math.signum(v4bDistance));
            pitchV4B.setPosition(initialPitchV4BPosition+ pitchV4bMotionState.position*Math.signum(pitchV4BDistance));
        }

        @Override
        public boolean finished() {
            if(Math.abs(rightV4B.getPosition()-v4bTargetPosition)<0.01 && Math.abs(pitchV4B.getPosition()-pitchV4BTargetPosition)<0.01)
            {
                rightV4B.setPosition(v4bTargetPosition);
                leftV4B.setPosition(v4bTargetPosition+servoOffset);
                pitchV4B.setPosition(pitchV4BTargetPosition);
                return true;
            }
            return false;
        }

        @Override
        public Set<Subsystem> requires() {
            Set<Subsystem> set = new HashSet<Subsystem>();
            set.add(IntakeSubsystem.this);
            return set;
        }
    }

    public Command telemetry()
    {
        return Command.builder()
                .update(()->{

                })
                .requires(this)
                .build();
    }



    // Manual
    public static class Manual {
      public double intakePosition=liftedPosition[0];
      public double pitchPosition=liftedPosition[1];
      public double servoOffset=IntakeSubsystem.servoOffset;
      public double leftIntakePosition=grabberPositions[0][1];
      public double rightIntakePosition=grabberPositions[0][0];
      public double intakePower=0.0;
    };

    public static Manual manual = new Manual();

    public Command manual()
    {
        return Command.builder()
                .update(()->{
                    leftV4B.setPosition(manual.intakePosition+manual.servoOffset);
                    rightV4B.setPosition(manual.intakePosition);
                    pitchV4B.setPosition(manual.pitchPosition);
                    leftIntake.setPosition(manual.leftIntakePosition);
                    rightIntake.setPosition(manual.rightIntakePosition);
                    intake.setPower(manual.intakePower);
                })
                .requires(this)
                .build();
    }

}
