package com.team9470.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Constants;
import com.team9470.Ports;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class Climber extends SubsystemBase {

    private static Climber instance;
    private final TalonFX climberMotor;
    private final TalonFX climberMotorFollower;

    // --- Control objects ---
    // Use MotionMagic for smooth motion to a target angle.
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    // When homing, we command a fixed voltage output.
    private final VoltageOut homingVoltage = new VoltageOut(Constants.AlgaeConstants.HOMING_OUTPUT);

    // --- Status Signals (for sensor feedback) ---
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Double> setpointSignal;
    private final StatusSignal<Double> setpointVelocitySignal;
    private final StatusSignal<Double> errorSignal;

    // --- Homing state machine ---
    private enum HomingState {
        IDLE,       // Normal operation
        HOMING,     // Running homing (moving toward the limit)
    }
    private Climber.HomingState homingState = Climber.HomingState.HOMING;

    // --- Target angle ---
    private Angle targetAngle = Constants.AlgaeConstants.HOMING_ANGLE;

    // --- Periodic I/O container ---
    public static class PeriodicIO {
        // Inputs
        public Time timestamp;
        public Angle position;                 // Measured pivot angle
        public AngularVelocity velocity;       // Measured angular velocity
        public Current current;                // Pivot motor current
        public Voltage voltage;                // Pivot motor voltage
        public Angle closedLoopError;          // Error (in degrees) between setpoint and actual

        // Outputs / Telemetry
        public Angle goal;                     // Desired target angle
        public Angle setpointAngle;            // Commanded setpoint (from closed-loop)
        public double setpointRaw;             // Raw setpoint value (degrees)
        public double setpointVelocityRaw;     // Raw setpoint velocity (deg/s)
        public AngularVelocity setpointAngularVelocity; // Converted setpoint velocity
        public Climber.HomingState homingState;        // Current homing state
    }
    private final Climber.PeriodicIO periodicIO = new Climber.PeriodicIO();



    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public Climber() {
        // Create the pivot and roller TalonFX instances.
        climberMotor = TalonFXFactory.createDefaultTalon(Ports.CLIMBER_MAIN);
        climberMotorFollower = TalonFXFactory.createPermanentFollowerTalon(
                Ports.CLIMBER_FOLLOWER, Ports.CLIMBER_MAIN, true);

        // Apply configuration (using your constant-provided configurations).
        TalonUtil.applyAndCheckConfiguration(climberMotor, Constants.ClimberConstants.getMainConfig());
        TalonUtil.applyAndCheckConfiguration(climberMotorFollower, Constants.ClimberConstants.getFollowerConfig());

        // Set up sensor status signals (refresh at 50 Hz with 0.1 sec latency tolerance).
        Frequency refreshRate = Hertz.of(50);
        positionSignal = climberMotor.getPosition();
        positionSignal.setUpdateFrequency(refreshRate, 0.1);
        velocitySignal = climberMotor.getVelocity();
        velocitySignal.setUpdateFrequency(refreshRate, 0.1);
        errorSignal = climberMotor.getClosedLoopError();
        errorSignal.setUpdateFrequency(refreshRate, 0.1);
        currentSignal = climberMotor.getStatorCurrent();
        currentSignal.setUpdateFrequency(refreshRate, 0.1);
        voltageSignal = climberMotor.getMotorVoltage();
        voltageSignal.setUpdateFrequency(refreshRate, 0.1);
        setpointSignal = climberMotor.getClosedLoopReference();
        setpointSignal.setUpdateFrequency(refreshRate, 0.1);
        setpointVelocitySignal = climberMotor.getClosedLoopReferenceSlope();
        setpointVelocitySignal.setUpdateFrequency(refreshRate, 0.1);
    }


    @Override
    public void periodic() {
        readPeriodicInputs();

        // Homing: if the subsystem needs homing, run the state machine.
        updateHomingLogic();
        periodicIO.homingState = homingState;

        writePeriodicOutputs();
        logTelemetry();
    }

    /** Read sensor values into our PeriodicIO structure. */
    private void readPeriodicInputs() {
        periodicIO.timestamp = Seconds.of(Timer.getFPGATimestamp());
        periodicIO.position = positionSignal.asSupplier().get();
        periodicIO.velocity = velocitySignal.asSupplier().get();
        periodicIO.current = currentSignal.asSupplier().get();
        periodicIO.voltage = voltageSignal.asSupplier().get();
        periodicIO.closedLoopError = Rotations.of(errorSignal.asSupplier().get());
        periodicIO.goal = targetAngle;
        periodicIO.setpointAngle = Rotations.of(setpointSignal.asSupplier().get());
        periodicIO.setpointRaw = setpointSignal.asSupplier().get();
        periodicIO.setpointVelocityRaw = setpointVelocitySignal.asSupplier().get();
        periodicIO.setpointAngularVelocity = RotationsPerSecond.of(setpointVelocitySignal.asSupplier().get());

    }

    /** Run homing state machine logic. */
    private void updateHomingLogic() {
        if (homingState == Climber.HomingState.HOMING) {
            // When homing, if the pivot is nearly stalled and the current is high,
            // assume the arm has hit the hard stop.
            boolean velocityStalled = Math.abs(periodicIO.velocity.in(DegreesPerSecond)) < 0.5;
            boolean currentTooHigh = periodicIO.current.gte(Constants.AlgaeConstants.HOMING_THRESHOLD);
            if (velocityStalled && currentTooHigh) {
                // Zero the sensor at the homing limit.
                climberMotor.setPosition(Constants.AlgaeConstants.HOMING_ANGLE);
                homingState = Climber.HomingState.IDLE;
            }
        }
    }

    /** Write outputs (commands) to the pivot motor. */
    private void writePeriodicOutputs() {
        if (homingState == Climber.HomingState.HOMING) {
            // While homing, command a fixed voltage.
            climberMotor.setControl(homingVoltage);
            periodicIO.setpointAngle = Rotations.of(0);
        } else {
            // Normal motion magic control to the target angle.
//            System.out.println("target angle: " + targetAngle.in(Degrexes));
            climberMotor.setControl(
                    motionMagic.withPosition(targetAngle)
                            .withSlot(0)
                            .withEnableFOC(true));
        }
    }

    /** Log telemetry values to SmartDashboard. */
    private void logTelemetry() {
        SmartDashboard.putNumber("Climber/Position_rot", periodicIO.position.in(Rotations));
        SmartDashboard.putNumber("Climber/Velocity_rps", periodicIO.velocity.in(RotationsPerSecond));
        SmartDashboard.putNumber("Climber/Current_A", periodicIO.current.in(Amps));
        SmartDashboard.putNumber("Climber/Voltage_V", periodicIO.voltage.in(Volts));
        SmartDashboard.putNumber("Climber/ClosedLoopError_deg", periodicIO.closedLoopError.in(Rotations));
        SmartDashboard.putString("Climber/HomingState", periodicIO.homingState.toString());
        SmartDashboard.putNumber("Climber/Goal_deg", periodicIO.goal.in(Rotations));
        SmartDashboard.putNumber("Climber/Setpoint_deg", periodicIO.setpointAngle.in(Rotations));
    }

    // --- Public Methods ---

    /** Set the desired target angle for the pivot. */
    public void setTargetAngle(Angle angle) {
        targetAngle = angle;
    }

    /** Get the current pivot angle. */
    public Angle getAngle() {
        return periodicIO.position;
    }

    /** Get the current angular velocity. */
    public AngularVelocity getAngularVelocity() {
        return periodicIO.velocity;
    }

    /** Get the pivot motor current. */
    public Current getPivotCurrent() {
        return periodicIO.current;
    }

    // --- Command methods ---

    /**
     * Returns a command that moves the arm to the specified angle.
     * The command is finished when the current angle is within a tolerance.
     */
    public Command getMoveToAngleCommand(Angle angle) {
        return new Command() {
            @Override
            public void execute() {
                setTargetAngle(angle);
            }
            @Override
            public boolean isFinished() {
                return getAngle().isNear(angle, Degrees.of(3));
            }

        };
    }

    public Command stow(){
        return getMoveToAngleCommand(Constants.ClimberConstants.STOW);
    }

    public Command clear() {
        return getMoveToAngleCommand(Constants.ClimberConstants.CLEAR);
    }

    public Command deploy() {
        return getMoveToAngleCommand(Constants.ClimberConstants.DEPLOY);
    }

}
