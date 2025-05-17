package com.team9470.commands;

import com.team9470.Constants;
import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.Set;

import static edu.wpi.first.units.Units.Meters;

public class AutoScoring {

    private CoralObjective coralObjective = AutoScoring.CoralObjective.NONE;
    private final Swerve drivetrain;

    public AutoScoring(Swerve drivetrain){
        this.drivetrain = drivetrain;
    }

    public Command autoScore(Superstructure superstructure) {
        return new DeferredCommand(() -> {
            Command driveToScore = new DriveToPose(coralObjective::getScoringPose, drivetrain)
                    .alongWith(
                            new WaitUntilCommand(() -> closeEnough(coralObjective, Constants.DriverAssistConstants.RAISE_DISTANCE))
                                    .andThen(superstructure.waitForIntake().asProxy())
                                    .andThen(superstructure.raise(coralObjective.level).asProxy())
                    );
            System.out.println("Scoring with coralobjective: " + coralObjective);
            return driveToScore;
        }, Set.of(drivetrain)).asProxy().andThen(superstructure.getCoral().scoreCommand().asProxy());
    }

    public static Command autoScore(Superstructure superstructure, CoralObjective objective, Swerve drivetrain) {
        // First drive to the scoring position while raising the superstructure.
        Command driveToScore = new DriveToPose(objective::getScoringPose, drivetrain)
                .alongWith(
                        new WaitUntilCommand(() -> closeEnough(objective, Constants.DriverAssistConstants.RAISE_DISTANCE))
                                .andThen(superstructure.waitForIntake().asProxy())
                                .andThen(superstructure.raise(objective.level))
                );
        if(objective.level == 1){
            return driveToScore.andThen(superstructure.getCoral().scoreSlow().asProxy());
        } else return driveToScore.andThen(superstructure.getCoral().scoreCommand().asProxy());
    }

    public static Command autoScoreWithTimeout(Superstructure superstructure, CoralObjective objective, Swerve drivetrain) {
        // First drive to the scoring position while raising the superstructure.
        Command driveToScore = new DriveToPose(objective::getScoringPose, drivetrain)
                .alongWith(
                        new WaitUntilCommand(() -> closeEnough(objective, Constants.DriverAssistConstants.RAISE_DISTANCE))
                                .andThen(superstructure.raise(objective::level))
                );
        return driveToScore.andThen(superstructure.getCoral().scoreCommand().withTimeout(1).until(() -> !superstructure.getCoral().hasCoral()).asProxy());
    }

    public static Command autoScoreStraight(Superstructure superstructure, CoralObjective objective, Swerve drivetrain) {
        // First drive to the scoring position while raising the superstructure.
        Command driveToScore = new DriveToPose(objective::getScoringPose, drivetrain, true)
                .alongWith(
                        new WaitUntilCommand(() -> closeEnough(objective, Constants.DriverAssistConstants.RAISE_DISTANCE.times(2)))
                                .andThen(superstructure.raise(objective::level))
                );
        return driveToScore.andThen(superstructure.getCoral().scoreCommand().withTimeout(1).until(() -> !superstructure.getCoral().hasCoral()).asProxy());
    }

    public Command autoScoreNoDrive(Superstructure superstructure) {
        return new DeferredCommand(() -> {
            if(coralObjective.level == 1){
                return superstructure.raise(coralObjective.level).andThen(superstructure.getCoral().scoreSlow());
            } else return superstructure.raise(coralObjective.level).andThen(superstructure.score());
        }, Set.of());
    }

    private Pose2d computeDriveBackPose(Pose2d scoringPose) {
        // Drive backward 1 meter relative to the scoring pose.
        Translation2d backOffset = new Translation2d(-.5, 0.0).rotateBy(scoringPose.getRotation());
        return new Pose2d(scoringPose.getTranslation().plus(backOffset), scoringPose.getRotation());
    }

    public Command driveBack(){
        Pose2d scoringPose = coralObjective.getScoringPose();
        Pose2d driveBackPose = computeDriveBackPose(scoringPose);

        return new DriveToPose(() -> driveBackPose, drivetrain);

    }

    private static boolean closeEnough(CoralObjective objective, Distance distance){
        Pose2d scoringPose = objective.getScoringPose();
        Pose2d currentPose = Swerve.getInstance().getPose();
        return currentPose.getTranslation().getDistance(scoringPose.getTranslation()) < distance.in(Meters);
    }

    // Update the reef value used for automatic level selection.
    public void setBranch(int branchId) {
        coralObjective = coralObjective.updateBranchId(branchId);
        System.out.println("Branch ID: " + branchId);
        SmartDashboard.putNumber("AutoScoring/Branch ID", branchId);
    }

    public void setLevel(int level) {
        coralObjective = coralObjective.updateLevel(level);
        SmartDashboard.putString("AutoScoring/Level: ", "L" + level);
    }

    public void updateClosestReefPos() {
        Pose2d[] reefPoses = Constants.DriverAssistConstants.getReefPositions();
        // Get the current robot pose at initialization.
        Pose2d currentPose = Swerve.getInstance().getPose();

        // Find the closest reef pose.
        double shortestDistance = Double.MAX_VALUE;
        int closestPoseId = -1;
        for (int i = 0; i < 12; i++) {
            Pose2d pose = reefPoses[i];
            double distance = currentPose.getTranslation().getDistance(pose.getTranslation());
            if (distance < shortestDistance) {
                shortestDistance = distance;
                closestPoseId = i;
            }
        }
        setBranch(closestPoseId);

    }

    // Starting facing the driver station, moving counterclockwise
    public record CoralObjective(int branchId, int level){
        public static final CoralObjective NONE = new CoralObjective(0, 0);

        public Pose2d getScoringPose(){
            if (level == 1) return Constants.DriverAssistConstants.getL1Pose(this);
            return Constants.DriverAssistConstants.getReefPositions()[branchId];
        }

        public int getFace(){
            return (branchId) / 2;
        }

        public int getAlgaeLevel(){
            // starting with (0, 1) = 3, then (2, 3) = 2, each reef alternates level between 2 and 3
            return (branchId)/2 % 2 == 0 ? 3 : 2;
        }

        public CoralObjective updateBranchId(int branchId){
            return new CoralObjective(branchId, level);
        }

        public CoralObjective updateLevel(int level){
            return new CoralObjective(branchId, level);
        }
    }
}
