package com.team9470.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.DelayedBoolean;
import com.team9470.Constants.CoralConstants;
import com.team9470.Ports;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

/**
 * By default, the coral manipulator runs `coastUnless()`. This runs the coral manipulator unless
 * a coral is detected, or another command overrides it.
 */
public class CoralManipulator extends SubsystemBase {
    /** ELECTRONICS */
    private final TalonFX coralMotor = TalonFXFactory.createDefaultTalon(Ports.CORAL_INTAKE);
    private final TalonFX funnelMotor = TalonFXFactory.createDefaultTalon(Ports.FUNNEL);
    private final DigitalInput coralSensor = new DigitalInput(Ports.CORAL_BREAK);

    /** ensures coral stops at the right position in coral manipulator, not just at the instant it gets detected.
     * (tune `Constants.CoralConstants.BREAK_TIMEOUT` to adjust how far after a rising edge to wait
     * until `coralBreak` becomes true) */
    private final DelayedBoolean coralBreak = new DelayedBoolean(Timer.getFPGATimestamp(), CoralConstants.BREAK_TIMEOUT, sensorTrue());

    private final LEDs leds = LEDs.getInstance();

    public CoralManipulator() {
        coralMotor.getConfigurator().apply(CoralConstants.getMotorConfig());
        funnelMotor.getConfigurator().apply(CoralConstants.getMotorConfig());
        setDefaultCommand(coastUnless());
    }

    @Override
    public void periodic() {
        // updates the delayed boolean with whether the coral is in the manipulator
        coralBreak.update(Timer.getFPGATimestamp(), sensorTrue());

        leds.hasCoral = sensorTrue();

        SmartDashboard.putBoolean("CoralManipulator/BeamBreak", sensorTrue());
        SmartDashboard.putBoolean("CoralManipulator/HasCoral", hasCoral());
    }

    /**
     * The beambreak is true if the beambreak is NOT "broken" (i.e. something is NOT detected)
     * The beambreak returns false if the beambreak IS "broken" (i.e. something IS detected)
     * This function should NOT be used directly -- use the DelayedBoolean to ensure the coral manipulator stops
     * only a bit after the coral is detected
     *
     * @return Whether coral is currently in the intake or not
     */
    public boolean sensorTrue() {
        return !coralSensor.get();
    }

    /**
     * A wrapper function for `sensorTrue()` that implements `DelayedBoolean` functionality
     *
     * @return Whether coral is PROPERLY in the intake or not
     */
    public boolean hasCoral() {
        return coralBreak.update(Timer.getFPGATimestamp(), sensorTrue());
    }

    /**
     * Runs the coral manipulator unless a coral is detected, or another command overrides it.
     *
     * @return The command to run this functionality
     */
    public Command coastUnless() {
        return this.run(() -> {
            if (hasCoral()) {
                coralMotor.setVoltage(CoralConstants.HOLD_SPEED.in(Volts));
                funnelMotor.stopMotor();
            } else {
                coralMotor.setVoltage(CoralConstants.COAST_SPEED.in(Volts));
                if (SmartDashboard.getNumber("Elevator/Position_m", 0) < CoralConstants.FUNNEL_SPEED_THRESHOLD.in(Meters))
                    funnelMotor.setVoltage(CoralConstants.FUNNEL_SPEED.in(Volts));
            }
        });
    }

    /**
     * DRIVER CONTROLLED COMMAND --> runs the coral manipulator to score
     */
    public Command scoreCommand(){
        return this.run(() -> {
            coralMotor.setVoltage(CoralConstants.TAKE_IN_SPEED.in(Volts));
            System.out.println("ATTEMPTING TO SCORE");
        });
    }

    public Command scoreSlow(){
        return this.run(() -> {
            coralMotor.setVoltage(CoralConstants.TAKE_IN_SPEED.in(Volts)/2);
            System.out.println("ATTEMPTING TO SCORE");
        });
    }

    /**
     * DRIVER CONTROLLED COMMAND --> runs the coral manipulator in reverse, either to remove a stuck coral
     * or if the coral has gone too far into the coral manipulator
     */
    public Command reverseCommand() {
        return this.runEnd(
            // reverses coral!
            () -> coralMotor.setVoltage(-CoralConstants.TAKE_IN_SPEED.in(Volts))
            // brakes the motor at the end, so the coral doesn't just end up misaligning but in reverse
            , coralMotor::stopMotor
        );
    }

    public Command scoreAndFunnel() {
        return this.run(
                () -> {
                    coralMotor.setVoltage(CoralConstants.TAKE_IN_SPEED.in(Volts));
                    funnelMotor.setVoltage(CoralConstants.FUNNEL_SPEED.in(Volts));
                }
        );
    }
}
//:3