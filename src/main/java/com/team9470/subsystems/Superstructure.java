package com.team9470.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.*;

import java.util.Set;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
    private final Elevator elevator;
    private final CoralManipulator coral;
    private final AlgaeArm algae;
    private final Climber climber;
    private final FunnelControl funnelControl;
    private final LEDs leds;



    public Superstructure(Mechanism2d mech) {
        this.elevator = new Elevator(mech);
        this.coral = new CoralManipulator();
        this.algae = new AlgaeArm(elevator.getElevatorLigament());
        this.leds = LEDs.getInstance();
        this.climber = new Climber();
        this.funnelControl = new FunnelControl();
    }

    // Returns a command to reverse the coral manipulator.
    public Command reverseCoral() {
        return coral.reverseCommand();
    }


    // Stow algae in alternate position (stow location 2) for coral scoring:
    // Raise elevator to L2 and move algae below the coral manipulator.
    public Command algaeUp() {
        return algae.up();
    }

    public Command algaeDown() {
        return algae.down();
    }

    public Command raise(int level){
        return elevator.getLevelCommand(level);
    }

    public Command raise(Supplier<Integer> getLevel){
        return elevator.getLevelCommand(getLevel.get());
    }

    
    public Command score() {
        return coral.scoreCommand();
    }

    // Trigger the algae arm’s homing routine.
    public Command triggerAlgaeHoming() {
        return new InstantCommand(algae::triggerHoming);
    }

    public Command waitForIntake() {
        return new WaitUntilCommand(coral::hasCoral);
    }

    public Command funnelOut() {
        return funnelControl.runOut();
    }

    enum ClimberState {
        CLEARING,
        DEPLOY,
        STOW;

        public static ClimberState next(ClimberState state) {
            switch (state) {
                case STOW:
                    return CLEARING;
                case DEPLOY:
                    return STOW;
                case CLEARING:
                    return DEPLOY;
                default:
                    throw new IllegalArgumentException("Invalid ClimberState: " + state);
            }
        }
    }

    private ClimberState climberState = ClimberState.STOW;

    public Command climberAction() {
        return new DeferredCommand(() -> {
            Command action;
            climberState = ClimberState.next(climberState);
            switch (climberState) {
                case STOW:
                    action = climber.stow();
                    break;
                case CLEARING:
                    action = climber.clear();
                    break;
                case DEPLOY:
                    action = climber.deploy();
                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + climberState);
            }

            return action;
        }, Set.of(climber));
    }




    // Accessors for the individual subsystems.
    public Elevator getElevator() {
        return elevator;
    }

    public CoralManipulator getCoral() {
        return coral;
    }

    public AlgaeArm getAlgae() {
        return algae;
    }

    public LEDs getLEDs() {
        return leds;
    }

    public Climber getClimber() {
        return climber;
    }

    public FunnelControl getFunnelControl() {
        return funnelControl;
    }

    public Command scoreAndFunnel() {
        return coral.scoreAndFunnel();
    }
}