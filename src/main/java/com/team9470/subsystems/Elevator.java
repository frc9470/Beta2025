package com.team9470.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Constants;
import com.team9470.Constants.ElevatorConstants;
import com.team9470.Ports;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Constants.ElevatorConstants.*;
import static edu.wpi.first.units.Units.*;

/**
 * Example Elevator subsystem that:
 * 1) Stores position/velocity in meters.
 * 2) Has a simple homing state machine.
 * 3) Logs telemetry to SmartDashboard.
 */
public class Elevator extends SubsystemBase {

    private final TalonFX elevatorMotor;
    private final TalonFX elevatorMotorFollower;

    // Control objects
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    private final VoltageOut homingVoltage = new VoltageOut(ElevatorConstants.HOMING_OUTPUT);

    // Status Signal Objects
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Double> setpointPositionSignal;
    private final StatusSignal<Double> setpointVelocitySignal;
    private final StatusSignal<Double> errorSignal;

    // Conversion factor: how many rotations per meter?
    // distancePerRotation ~ 0.2794 m, so ~3.58 rotations per meter
    private final double rotationsPerMeter =
            1.0 / DIST_PER_ROTATION.in(Meters);

    // Homing logic
    private boolean needsHoming = true;
    private HomingState homingState = HomingState.IDLE;

    // Our "goal" position in meters
    private Distance targetPosition = L0;

    // Keep track of time when homing started
    private Time homingStartTime = Seconds.of(0);

    // PeriodicIO for reading/writing
    private final PeriodicIO periodicIO = new PeriodicIO();


    public MechanismLigament2d getElevatorLigament() {
        return elevatorLigament;
    }


    private enum HomingState {
        IDLE,       // Not homing
        HOMING,     // Running motor downward
        HOMED       // We found bottom and zeroed
    }

    public TalonFXSimState mainTalonFXSim;
    public TalonFXSimState followerTalonFXSim;

    private final ElevatorSim elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            Constants.ElevatorConstants.GEAR_RATIO,
            Constants.ElevatorConstants.MASS,
            Constants.ElevatorConstants.DRUM_RADIUS,
            Constants.ElevatorConstants.HOME_POSITION.in(Meter),
            Constants.ElevatorConstants.L4.in(Meter),
            true,
            Constants.ElevatorConstants.HOME_POSITION.in(Meter),
            0.01, 0.0
        );

    private final Mechanism2d mechanism;
    private final MechanismRoot2d elevatorRoot;
    private final MechanismLigament2d elevatorLigament;

    /**
     * Container for inputs and outputs that we want to log.
     * The idea is to gather all I/O in one place.
     */
    public static class PeriodicIO {
        // ---------------- Inputs ----------------
        public Time timestamp;
        public Distance positionMeters;         // Actual elevator position (m)
        public LinearVelocity velocityMps;      // Actual elevator velocity (m/s)
        public Current current;                 // Stator current
        public Voltage voltage;
        public Distance closedLoopError;        // difference between on-loop SETPOINT & actual

        // ---------------- Outputs / Telemetry ----------------
        public Distance goal;                         // Overall goal of the system
        public Distance setpointPositionMeters;        // The PER-LOOP commanded position in meters
        public double setpointPositionRotations;       // The PER-LOOP commanded position in rotations
        public double setpointVelocityRotPerSec;       // The commanded velocity in rotations/sec
        public LinearVelocity setpointVelocityMps;     // The commanded velocity in m/s
        public HomingState homingState;               // Current homing state
    }

    public Elevator(Mechanism2d mechanism) {
        elevatorMotor = TalonFXFactory.createDefaultTalon(Ports.ELEVATOR_MAIN);
        elevatorMotorFollower = TalonFXFactory.createPermanentFollowerTalon(
                Ports.ELEVATOR_FOLLOWER, Ports.ELEVATOR_MAIN, true);

        TalonUtil.applyAndCheckConfiguration(elevatorMotor, ElevatorConstants.ElevatorFXConfig());
        TalonUtil.applyAndCheckConfiguration(elevatorMotorFollower, ElevatorConstants.ElevatorFXConfigFollower());
        
        Frequency refreshRate = Hertz.of(50);

        positionSignal = elevatorMotor.getPosition();
        positionSignal.setUpdateFrequency(refreshRate, 0.1);
        velocitySignal = elevatorMotor.getVelocity();
        velocitySignal.setUpdateFrequency(refreshRate, 0.1);
        errorSignal = elevatorMotor.getClosedLoopError();
        errorSignal.setUpdateFrequency(refreshRate, 0.1);

        currentSignal = elevatorMotor.getStatorCurrent();
        currentSignal.setUpdateFrequency(refreshRate, 0.1);
        voltageSignal = elevatorMotor.getMotorVoltage();
        voltageSignal.setUpdateFrequency(refreshRate, 0.1);

        setpointPositionSignal = elevatorMotor.getClosedLoopReference();
        setpointPositionSignal.setUpdateFrequency(refreshRate, 0.1);
        setpointVelocitySignal = elevatorMotor.getClosedLoopReferenceSlope();
        setpointVelocitySignal.setUpdateFrequency(refreshRate, 0.1);

        // Create a 2d mechanism for visualization.
        // Dimensions (width, height) are arbitrary units; adjust as needed.
        this.mechanism = mechanism;
        // Set a root; here (100,0) places it at the bottom center.
        elevatorRoot = mechanism.getRoot("Elevator", 2.5, 0);
        // Append a ligament that represents the elevator carriage.
        // A vertical ligament (angle=90) whose length you update based on position.
        elevatorLigament = elevatorRoot.append(new MechanismLigament2d("Carriage", elevatorSim.getPositionMeters() + 0.58, 90));

        // Publish the visualization to Shuffleboard (or SmartDashboard)
    }

    @Override
    public void periodic() {
        // 1) Read inputs
        readPeriodicInputs();

        // 2) Handle state machine logic (homing & normal)
        if (needsHoming) {
            updateHomingLogic();
        } else {
            homingState = HomingState.IDLE;
        }
        // Store the homing state in periodicIO for telemetry
        periodicIO.homingState = homingState;

        // 3) Write outputs (calculate setpoints and command the motor)
        writePeriodicOutputs();

        // 4) Log data to SmartDashboard (or any other telemetry sink)
        logTelemetry();
    }

    public void simulationPeriodic() {
        mainTalonFXSim = elevatorMotor.getSimState();
        mainTalonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        followerTalonFXSim = elevatorMotor.getSimState();
        followerTalonFXSim.Orientation = ChassisReference.Clockwise_Positive;

        mainTalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = mainTalonFXSim.getMotorVoltage();

        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        elevatorSim.setInputVoltage(motorVoltage);

         // Next, we update it. The standard loop time is 20ms.
         elevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        mainTalonFXSim.setRawRotorPosition(elevatorSim.getPositionMeters() * rotationsPerMeter * GEAR_RATIO);
        mainTalonFXSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * rotationsPerMeter * GEAR_RATIO);
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps())
        );
    }

    // ------------------ Public Methods ------------------

    /** Command the elevator to a given (meter) setpoint. */
    public void setPosition(Distance position) {
        targetPosition = position;
    }

    /** Get the elevator’s current position in meters. */
    public Distance getPosition() {
        return periodicIO.positionMeters;
    }

    /** Return true if we are in the middle of homing. */
    public boolean isHoming() {
        return homingState == HomingState.HOMING;
    }

    // Example: check if we might be stalling
    public boolean isStalling() {
        return periodicIO.current.in(Amps) >= 10;
    }

    // ------------------ Internal (Periodic) Methods ------------------

    /** Reads sensor data from the motor and updates periodicIO fields. */
    private void readPeriodicInputs() {
        periodicIO.timestamp = Seconds.of(Timer.getFPGATimestamp());

        // The motor’s position is in rotations. Convert to meters.
        double rotations = positionSignal.asSupplier().get().in(Rotations);
        periodicIO.positionMeters = Meters.of(rotations / rotationsPerMeter);

        // The motor’s velocity is in rotations/sec. Convert to m/s.
        double rotPerSec = velocitySignal.asSupplier().get().in(RotationsPerSecond);
        periodicIO.velocityMps = MetersPerSecond.of(rotPerSec / rotationsPerMeter);

        // Current
        periodicIO.current = currentSignal.asSupplier().get();
        periodicIO.voltage = voltageSignal.asSupplier().get();

        // For closed-loop error (just for debugging),
        // we can compare target vs. actual in meters:
        periodicIO.closedLoopError = DIST_PER_ROTATION.times(errorSignal.asSupplier().get());

        // For telemetry
        periodicIO.goal = targetPosition;
        periodicIO.setpointPositionMeters = DIST_PER_ROTATION.times(setpointPositionSignal.asSupplier().get());
        periodicIO.setpointPositionRotations = setpointPositionSignal.asSupplier().get();
        periodicIO.setpointVelocityRotPerSec = setpointVelocitySignal.asSupplier().get();
        periodicIO.setpointVelocityMps = DIST_PER_ROTATION.times(setpointVelocitySignal.asSupplier().get()).per(Seconds);
    }

    /** Runs the homing state machine if needed. */
    private void updateHomingLogic() {
        switch (homingState) {
            case IDLE:
                // Example condition: only start homing if the elevator
                // is basically at the commanded position (meaning not in use).

                boolean timeOut = periodicIO.timestamp
                        .minus(homingStartTime)
                        .gt(ElevatorConstants.HOMING_TIMEOUT);
                if (L0.isNear(periodicIO.positionMeters, Meters.of(0.02)) && timeOut) {
                    homingState = HomingState.HOMING;
                    homingStartTime = periodicIO.timestamp;
                }
                break;

            case HOMING:
                // 1) If velocity is near zero or current is high, we assume bottom
                // 2) Or we time out
                boolean velocityStalled =
                        Math.abs(periodicIO.velocityMps.in(MetersPerSecond)) < 0.01;
                boolean currentTooHigh = isStalling();

                if (velocityStalled && currentTooHigh) {
                    // We consider ourselves at the bottom => zero the sensor
                    elevatorMotor.setPosition(0);
                    homingState = HomingState.HOMED;
                }
                break;

            case HOMED:
                // Done homing. Turn off the need for homing, remain in IDLE next loop.
                needsHoming = false;
                homingState = HomingState.IDLE;
                break;
        }
    }

    /** Write outputs back to the motor(s). */
    private void writePeriodicOutputs() {
        if (homingState == HomingState.HOMING) {
            // Force a negative voltage to push elevator down
            elevatorMotor.setControl(homingVoltage);

            // For telemetry: set the "desired position/velocity" to something informative
            periodicIO.setpointPositionMeters = Meters.of(0);
            periodicIO.setpointPositionRotations = 0;
            periodicIO.setpointVelocityRotPerSec = 0;
            periodicIO.setpointVelocityMps = MetersPerSecond.of(0);
        } else {
            // Normal control
            // Convert target position (meters) to rotations:
            double targetRotations = targetPosition.in(Meters) * rotationsPerMeter;

            // Let DynamicMotionMagicVoltage handle the motion magic.
            // We set velocity + accel so it knows our motion constraints.
            elevatorMotor.setControl(
                    motionMagic
                            .withPosition(targetRotations)
                            .withSlot(0)
                            .withEnableFOC(true)
            );
        }

        // Once we think we’ve reached the setpoint, request a re-home
        // so next time we move from a known zero.
        // Or do it only if the setpoint > 0, etc.
        if (!needsHoming &&
                (homingState != HomingState.HOMING) &&
                periodicIO.closedLoopError.abs(Meters) < 0.01 &&
                Math.abs(periodicIO.velocityMps.in(MetersPerSecond)) < 0.01
                && targetPosition.gt(Meters.of(0))) {
            needsHoming = true;
        }
    }

    /** Publish key telemetry values to SmartDashboard. */
    private void logTelemetry() {
        // Actual sensor readings
        SmartDashboard.putNumber("Elevator/Position_m", periodicIO.positionMeters.in(Meters));
        SmartDashboard.putNumber("Elevator/Velocity_mps", periodicIO.velocityMps.in(MetersPerSecond));
        SmartDashboard.putNumber("Elevator/Current_A", periodicIO.current.in(Amps));
        SmartDashboard.putNumber("Elevator/Voltage", periodicIO.voltage.in(Volts));
        SmartDashboard.putNumber("Elevator/Error_m", periodicIO.closedLoopError.in(Meters));

        // Goal
        SmartDashboard.putNumber("Elevator/Goal", periodicIO.goal.in(Meters));

        // Desired setpoints
        SmartDashboard.putNumber("Elevator/Setpoint/Position_m", periodicIO.setpointPositionMeters.in(Meters));
        SmartDashboard.putNumber("Elevator/Setpoint/Position_rot", periodicIO.setpointPositionRotations);
        SmartDashboard.putNumber("Elevator/Setpoint/Velocity_rps", periodicIO.setpointVelocityRotPerSec);
        SmartDashboard.putNumber("Elevator/Setpoint/Velocity_mps", periodicIO.setpointVelocityMps.in(MetersPerSecond));

        // Homing info
        SmartDashboard.putString("Elevator/HomingState", periodicIO.homingState.toString());
        SmartDashboard.putBoolean("Elevator/NeedsHoming", needsHoming);


        // Mechanism 2D output
        elevatorLigament.setLength(Math.max(0.58, 0.58 + periodicIO.positionMeters.in(Meters)));
//        SmartDashboard.putData("Elevator/Mechanism", elevatorMechanism);


    }

    // ------------------ External (Command) Methods ------------------
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

    public Command getMoveToPositionCommand(Distance position) {
        return new Command() {
            @Override
            public void execute() {
                setPosition(position);
            }

            @Override
            public void end(boolean interrupted) {
//                setPosition(Meters.of(0));
            }

            @Override
            public boolean isFinished() {
                return getPosition().isNear(position, Meters.of(0.01));
            }
        };
    }

    /* command that doesn't wait for the elevator to reach the setpoint */
    public Command getMoveToPositionCommandNoWait(Distance position) {
        return new Command() {
            @Override
            public void execute() {
                setPosition(position);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    public Command L0() {
        return getMoveToPositionCommand(ElevatorConstants.L0);
    }

    public Command L1(){
        return getMoveToPositionCommand(ElevatorConstants.L1);
    }

    public Command L2(){
        return getMoveToPositionCommand(ElevatorConstants.L2);
    }

    public Command L3(){
        return getMoveToPositionCommand(ElevatorConstants.L3);
    }

    public Command L4(){
        return getMoveToPositionCommand(ElevatorConstants.L4);
    }
    public Command intake(){
        return getMoveToPositionCommand(ElevatorConstants.INTAKE);
    }


    public Command getLevelCommand(int level){
        return switch (level) {
            case 1 -> L1();
            case 2 -> L2();
            case 3 -> L3();
            case 4 -> L4();
            default -> L0();
        };
    }


}