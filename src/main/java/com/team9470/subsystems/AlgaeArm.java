package com.team9470.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Constants.AlgaeConstants;
import com.team9470.Ports;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class AlgaeArm extends SubsystemBase {

    // --- Hardware ---
    private final TalonFX pivotMotor;

    // --- Control objects ---
    // Use MotionMagic for smooth motion to a target angle.
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    // When homing, we command a fixed voltage output.
    private final VoltageOut homingVoltage = new VoltageOut(AlgaeConstants.HOMING_OUTPUT);

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
        HOMED       // Homing complete; sensor zeroed
    }
    private boolean needsHoming = true;
    private HomingState homingState = HomingState.HOMING;
    private Time homingStartTime = Seconds.of(0);

    // --- Target angle ---
    private Angle targetAngle = AlgaeConstants.HOMING_ANGLE;

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
        public HomingState homingState;        // Current homing state
    }
    private final PeriodicIO periodicIO = new PeriodicIO();

    // --- Simulation objects ---
    private final SingleJointedArmSim armSim;
    private TalonFXSimState pivotSimState;

    // --- Visualization (Mechanism2d) ---
    private final MechanismLigament2d armLigament;

    public AlgaeArm(MechanismLigament2d mechanism) {
        // Create the pivot and roller TalonFX instances.
        pivotMotor = TalonFXFactory.createDefaultTalon(Ports.ALGAE_PIVOT);

        // Apply configuration (using your constant-provided configurations).
        TalonUtil.applyAndCheckConfiguration(pivotMotor, AlgaeConstants.getPivotConfig());

        // Set up sensor status signals (refresh at 50 Hz with 0.1 sec latency tolerance).
        Frequency refreshRate = Hertz.of(50);
        positionSignal = pivotMotor.getPosition();
        positionSignal.setUpdateFrequency(refreshRate, 0.1);
        velocitySignal = pivotMotor.getVelocity();
        velocitySignal.setUpdateFrequency(refreshRate, 0.1);
        errorSignal = pivotMotor.getClosedLoopError();
        errorSignal.setUpdateFrequency(refreshRate, 0.1);
        currentSignal = pivotMotor.getStatorCurrent();
        currentSignal.setUpdateFrequency(refreshRate, 0.1);
        voltageSignal = pivotMotor.getMotorVoltage();
        voltageSignal.setUpdateFrequency(refreshRate, 0.1);
        setpointSignal = pivotMotor.getClosedLoopReference();
        setpointSignal.setUpdateFrequency(refreshRate, 0.1);
        setpointVelocitySignal = pivotMotor.getClosedLoopReferenceSlope();
        setpointVelocitySignal.setUpdateFrequency(refreshRate, 0.1);

        // Publish this subsystem to SmartDashboard.

        // --- Initialize simulation for the pivoting arm ---
        // Using a SingleJointedArmSim to simulate arm dynamics.
        armSim = new SingleJointedArmSim(
                // Simulated motor (adjust if necessary)
                DCMotor.getKrakenX60Foc(1),
                AlgaeConstants.GEAR_RATIO,                  // Gear ratio (motor:arm)
                AlgaeConstants.ARM_MASS.in(Kilogram),                          // Mass (kg)
                AlgaeConstants.ARM_LENGTH.in(Meter),              // Arm length (m)
                AlgaeConstants.MIN_ANGLE.in(Radians),             // Minimum angle (radians)
                AlgaeConstants.MAX_ANGLE.in(Radians),             // Maximum angle (radians)
                false,                                            // Simulate gravity
                -Math.PI / 2, // starting angle
                0.01, 0
        );

        // Scale the visual length (e.g., 1 meter = 10 mechanism units)
        armLigament = mechanism.append(new MechanismLigament2d("Arm", AlgaeConstants.ARM_LENGTH.in(Meter) * 3, -180));
    }

    @Override
    public void periodic() {
        readPeriodicInputs();

        // Homing: if the subsystem needs homing, run the state machine.
        if (needsHoming) {
            updateHomingLogic();
        } else {
            homingState = HomingState.IDLE;
        }
        periodicIO.homingState = homingState;

        writePeriodicOutputs();
        logTelemetry();
    }

    /**
     * Simulation periodic – to be called from the simulation loop.
     * <p>
     * This method updates our simulated sensor values using a SingleJointedArmSim and
     * updates our Mechanism2d visualization.
     */
    public void simulationPeriodic() {
        pivotSimState = pivotMotor.getSimState();
        pivotSimState.Orientation = ChassisReference.Clockwise_Positive;
        pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        double motorVoltage = pivotSimState.getMotorVoltage();

        // Update simulation with the applied voltage.
        armSim.setInputVoltage(motorVoltage);
        armSim.update(0.020); // assuming a 20ms loop

        // Convert simulated arm angle (radians) to motor rotations.
        double motorRotations = (armSim.getAngleRads() / (2 * Math.PI)) * AlgaeConstants.GEAR_RATIO;
        double motorAngularVelocity = (armSim.getVelocityRadPerSec() / (2 * Math.PI)) * AlgaeConstants.GEAR_RATIO;

        pivotSimState.setRawRotorPosition(motorRotations);
        pivotSimState.setRotorVelocity(motorAngularVelocity);

        // Update battery voltage simulation.
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps())
        );

        // Update visualization: set the ligament angle (in degrees).
        double armAngleDeg = Math.toDegrees(armSim.getAngleRads());
        armLigament.setAngle(armAngleDeg - 90);
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
        switch (homingState) {
            case IDLE:
                // Example: if the arm is near its target (and a timeout has passed), start homing.
                boolean timeOut = periodicIO.timestamp.minus(homingStartTime).gt(AlgaeConstants.HOMING_TIMEOUT);
                if (AlgaeConstants.HOMING_ANGLE.isNear(periodicIO.position, Degrees.of(5)) && timeOut) {
                    homingState = HomingState.HOMING;
                    homingStartTime = periodicIO.timestamp;
                }
                break;
            case HOMING:
                // When homing, if the pivot is nearly stalled and the current is high,
                // assume the arm has hit the hard stop.
                boolean velocityStalled = Math.abs(periodicIO.velocity.in(DegreesPerSecond)) < 0.5;
                boolean currentTooHigh = periodicIO.current.gte(AlgaeConstants.HOMING_THRESHOLD);
                if (velocityStalled && currentTooHigh) {
                    // Zero the sensor at the homing limit.
                    pivotMotor.setPosition(AlgaeConstants.HOMING_ANGLE);
                    // reset the simulation in sim
                    if(pivotSimState != null) {
                        armSim.setState(AlgaeConstants.HOMING_ANGLE.in(Radians), 0);
                    }
                    homingState = HomingState.HOMED;
                }
                break;
            case HOMED:
                // Once homed, clear the flag so normal control resumes.
                needsHoming = false;
                homingState = HomingState.IDLE;
                break;
        }
    }

    /** Write outputs (commands) to the pivot motor. */
    private void writePeriodicOutputs() {
        if (homingState == HomingState.HOMING) {
            // While homing, command a fixed voltage.
            pivotMotor.setControl(homingVoltage);
            periodicIO.setpointAngle = Degrees.of(0);
        } else {
            // Normal motion magic control to the target angle.
//            System.out.println("target angle: " + targetAngle.in(Degrexes));
            pivotMotor.setControl(
                    motionMagic.withPosition(targetAngle)
                            .withSlot(0)
                            .withEnableFOC(true));
        }
    }

    /** Log telemetry values to SmartDashboard. */
    private void logTelemetry() {
        SmartDashboard.putNumber("AlgaeArm/Position_deg", periodicIO.position.in(Degrees));
        SmartDashboard.putNumber("AlgaeArm/Velocity_dps", periodicIO.velocity.in(DegreesPerSecond));
        SmartDashboard.putNumber("AlgaeArm/Current_A", periodicIO.current.in(Amps));
        SmartDashboard.putNumber("AlgaeArm/Voltage_V", periodicIO.voltage.in(Volts));
        SmartDashboard.putNumber("AlgaeArm/ClosedLoopError_deg", periodicIO.closedLoopError.in(Degrees));
        SmartDashboard.putString("AlgaeArm/HomingState", periodicIO.homingState.toString());
        SmartDashboard.putNumber("AlgaeArm/Goal_deg", periodicIO.goal.in(Degrees));
        SmartDashboard.putNumber("AlgaeArm/Setpoint_deg", periodicIO.setpointAngle.in(Degrees));
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



    public void triggerHoming() {
        needsHoming = true;
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
                return getAngle().isNear(angle, Degrees.of(0.5));
            }

            @Override
            public void end(boolean interrupted) {
                triggerHoming();
            }
        };
    }

    /**
     * Returns a command that starts homing the arm.
     * The command is finished once the homing state indicates that homing is complete.
     */
    public Command getHomingCommand() {
        return new Command() {
            @Override
            public void execute() {
                needsHoming = true;
            }
            @Override
            public boolean isFinished() {
                return homingState == HomingState.HOMED;
            }
        };
    }


    public Command up(){
        return getMoveToAngleCommand(AlgaeConstants.STOW_UP);
    }

    public Command down() {
        return getMoveToAngleCommand(AlgaeConstants.STOW_DOWN);
    }

}