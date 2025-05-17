package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.team9470.subsystems.*;
import com.team9470.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;

import java.util.Set;

import static edu.wpi.first.units.Units.Degrees;

public class Autos extends SubsystemBase{
    private final AutoFactory autoFactory;
    private final AlgaeArm algaeArm;
    private final CoralManipulator coralManipulator;
    private final Elevator elevator;
    private final Swerve swerve;
    private final Superstructure superstructure;

    public Autos(Superstructure superstructure, Swerve swerve) {
        this.autoFactory = swerve.createAutoFactory((sample, isStart) -> {
        });
        this.algaeArm = superstructure.getAlgae();
        this.coralManipulator = superstructure.getCoral();
        this.elevator = superstructure.getElevator();
        this.swerve = swerve;
        this.superstructure = superstructure;
    }

    public static final double SCORING_DELAY = 0.3;
    public static final double INTAKE_DELAY = 0.3;
    private static final double ELEVATOR_DELAY = 0.7;

    public Command scoreL4(){
        return elevator.L4().andThen(
                coralManipulator.scoreCommand().withTimeout(SCORING_DELAY))
                .andThen(elevator.L0());
    }

    public Command scoreL4WaitLower(Command driveAway, double delay){
        return elevator.L4()
                .andThen(
                    coralManipulator.scoreCommand().withTimeout(SCORING_DELAY)
                )
                .andThen(
                        driveAway
                        .alongWith(
                                new WaitCommand(delay).andThen(elevator.L0())
                        )
                );

    }

    public Command scoreL4AutoWaitLower(Command driveAway, double delay, int branchID){
        return AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(branchID, 4), swerve).andThen(driveAway
                .alongWith(
                        new WaitCommand(delay).andThen(elevator.L0())
                ));
    }

    public Pose2d getSourcePose(boolean top){
        return AllianceFlipUtil.apply(
                top ? new Pose2d(1.52, 7.35, Rotation2d.fromDegrees(-54)) : new Pose2d(1.45, 0.73, Rotation2d.fromDegrees(54))
        );
    }

    public Command alignToSource(boolean top) {

        return new DriveToPose(() -> getSourcePose(top), swerve, true, 3).raceWith(superstructure.waitForIntake());
    }

    public Command alignToSourceWait(boolean top){
        return new DriveToPose(() -> getSourcePose(top), swerve, true, 10)
                .andThen(superstructure.waitForIntake().deadlineFor(new DriveToPose(() -> getSourcePose(top), swerve, true, -1)));

    }

    public Command scoreCoral() {
        return coralManipulator.scoreCommand();
    }

    public AutoRoutine getOneCoralMiddleAutoNormal(){
        AutoRoutine routine = autoFactory.newRoutine("1CMN");

        AutoTrajectory startToC3 = routine.trajectory("S-3");
        routine.active().onTrue(
                startToC3.resetOdometry().andThen(
                        scoreL4AutoWaitLower(new InstantCommand(), SCORING_DELAY, 7)
                                .andThen(superstructure.algaeDown())
                )

        );
        return routine;
    }

    public AutoRoutine getOneCoralMiddleAutoChoreo(){
        AutoRoutine routine = autoFactory.newRoutine("1CMC");
        AutoTrajectory startToC3 = routine.trajectory("S-3");

        routine.active().onTrue(
                startToC3.resetOdometry()
                        .andThen(startToC3.cmd())
        );

        startToC3.done().onTrue(
                scoreL4WaitLower(new InstantCommand(), SCORING_DELAY)
                        .andThen(superstructure.algaeDown())
        );
        return routine;
    }

    public AutoRoutine getOneCoralMiddleAutoAlign(){
        AutoRoutine routine = autoFactory.newRoutine("1CMA");

        routine.active().onTrue(
                AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(7, 4), swerve)
                        .andThen(Commands.parallel(superstructure.algaeDown(), elevator.L0()))
        );

        return routine;
    }
    private static final Pose2d START_TOP = new Pose2d(new Translation2d(7, 5), new Rotation2d(Degrees.of(240)));
    private static final Pose2d START_BOTTOM = new Pose2d(new Translation2d(7, 3), new Rotation2d(Degrees.of(120)));

    public AutoRoutine getFiveCoralTopAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("5CTA");
        // TODO: define odom start position and reset odometry to that position, analogous to "startToC5.resetOdometry()" choreo call

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScoreStraight(superstructure, new AutoScoring.CoralObjective(9, 4), swerve),
                        elevator.L0()
                                .alongWith(algaeArm.up())
                                .withDeadline(alignToSourceWait(true)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(10, 4), swerve),
                        elevator.L0()
                                .withDeadline(alignToSourceWait(true)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(11, 4), swerve),
                        elevator.L0()
                                .withDeadline(alignToSourceWait(true)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(11, 3), swerve),
                        elevator.L0()
                                .alongWith(algaeArm.down())
                                .withDeadline(alignToSourceWait(true)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(10, 3), swerve)
                )
        );

        return routine;
    }

    public AutoRoutine getFiveCoralBottomAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("5CBA");

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(4, 4), swerve),
                        elevator.L0()
                                .alongWith(algaeArm.up())
                                .withDeadline(alignToSourceWait(false)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, 4), swerve),
                        elevator.L0()
                                .withDeadline(alignToSourceWait(false)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, 4), swerve),
                        elevator.L0()
                                .withDeadline(alignToSourceWait(false)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, 3), swerve),
                        elevator.L0()
                                .alongWith(algaeArm.down())
                                .withDeadline(alignToSourceWait(false)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, 3), swerve)
                )
        );

        return routine;
    }

    public AutoRoutine getFiveCoralTopAutoAlignNoWait() {
        AutoRoutine routine = autoFactory.newRoutine("5CTA-NW");

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(9, 4), swerve),
                        elevator.L0().alongWith(algaeArm.up())
                                .withDeadline(alignToSource(true)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(10, 4), swerve),
                        elevator.L0().alongWith(algaeArm.down())
                                .withDeadline(alignToSource(true)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(11, 4), swerve),
                        elevator.L0()
                                .withDeadline(alignToSource(true)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(11, 2), swerve),
                        elevator.L0().alongWith(algaeArm.up())
                                .withDeadline(alignToSource(true)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(10, 2), swerve)
                )
        );

        return routine;
    }

    public AutoRoutine getFiveCoralBottomAutoAlignNoWait() {
        AutoRoutine routine = autoFactory.newRoutine("5CBA-NW");

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScoreStraight(superstructure, new AutoScoring.CoralObjective(4, 4), swerve),
                        elevator.L0().alongWith(algaeArm.up())
                                .withDeadline(alignToSource(false)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, 4), swerve),
                        elevator.L0().alongWith(algaeArm.down())
                                .withDeadline(alignToSource(false)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, 4), swerve),
                        elevator.L0()
                                .withDeadline(alignToSource(false)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, 3), swerve),
                        elevator.L0().alongWith(algaeArm.up())
                                .withDeadline(alignToSource(false)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, 3), swerve)
                )
        );

        return routine;
    }

    // basic leave auto
    public Command getBasicAutoCommand() {
        ChassisSpeeds initial_speed = new ChassisSpeeds(1, 0, 0);
        ChassisSpeeds final_speed = new ChassisSpeeds(0, 0, 0);

        return this.runEnd(
            () -> swerve.setChassisSpeeds(initial_speed),
            () -> swerve.setChassisSpeeds(final_speed)
        ).withTimeout(1);
    }

}
