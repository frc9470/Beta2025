package com.team9470.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.DelayedBoolean;
import com.team9470.Constants.CoralConstants;
import com.team9470.Ports;
import com.team9470.subsystems.LEDs;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;


/**
 * CoralManipulator - Pranesh
 * Subsystem Requirements:
 * - move wheels forward or backward with command
 * - use a sensor to detect if a coral is in the manipulator
 * - one motor
 */
public class CoralManipulator extends SubsystemBase {
    private final TalonFX coralMotor = TalonFXFactory.createDefaultTalon(Ports.CORAL_INTAKE);
    private final TalonFX funnelMotor = TalonFXFactory.createDefaultTalon(Ports.FUNNEL);
    private final DigitalInput coralSensor = new DigitalInput(Ports.CORAL_BREAK);
    private final DelayedBoolean coralBreak = new DelayedBoolean(Timer.getFPGATimestamp(), CoralConstants.BREAK_TIMEOUT, sensorTrue());

    // gets LEDs singleton
    

    public CoralManipulator() {
        setDefaultCommand(coastUnless());
    }

    @Override
    public void periodic() {
        LEDs.getInstance().hasCoral = hasCoral();

        coralBreak.update(Timer.getFPGATimestamp(), sensorTrue());
        if(!hasCoral())
            funnelMotor.setVoltage(CoralConstants.FUNNEL_SPEED.in(Volts));
        else funnelMotor.stopMotor();

        SmartDashboard.putBoolean("CoralManipulator/BeamBreak", sensorTrue());
        SmartDashboard.putBoolean("CoralManipulator/HasCoral", hasCoral());
    }

    // it works!
    /**
     * The beambreak is true if the beambreak is NOT "broken" (i.e. something is NOT detected)
     * The beambreak returns false if the beambreak IS "broken" (i.e. something IS detected)
     *
     * @return Whether coral is currently in the intake or not
     */
    public boolean sensorTrue() {
        return !coralSensor.get();
    }

    public boolean hasCoral() {
        return coralBreak.update(Timer.getFPGATimestamp(), sensorTrue());
    }

    public Command coastUnless(){
        return this.run(() -> {
            if(hasCoral()){
                coralMotor.setVoltage(CoralConstants.HOLD_SPEED.in(Volts));
            } else {
                coralMotor.setVoltage(CoralConstants.COAST_SPEED.in(Volts));
            }
        });
    }

    public Command scoreCommand(){
        return this.run(() -> coralMotor.setVoltage(CoralConstants.TAKE_IN_SPEED.in(Volts)));
    }


    public Command reverseCommand() {
        return this.runEnd(
                () -> coralMotor.setVoltage(-CoralConstants.TAKE_IN_SPEED.in(Volts))
                , coralMotor::stopMotor
        );
    }
}
