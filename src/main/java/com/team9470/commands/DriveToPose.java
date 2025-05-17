package com.team9470.commands;

import com.team9470.FieldConstants.Reef;
import com.team9470.TunerConstants;
import com.team9470.subsystems.Swerve;
import com.team9470.util.GeomUtil;
import com.team9470.util.LogUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class DriveToPose extends Command {
    private final Swerve drivetrain;
    private final Supplier<Pose2d> reefPoseSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;
    private Pose2d reefPose;
    private boolean noInterpolate;
    private double tolerance = 1;

    // PID controllers for X, Y and Theta

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(TunerConstants.maxVelocity, TunerConstants.maxAcceleration);
    TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(TunerConstants.maxAngularVelocity, TunerConstants.maxAngularAcceleration);

    private final ProfiledPIDController pidControllerX = new ProfiledPIDController(7, 0, 0, constraints);
    private final ProfiledPIDController pidControllerY = new ProfiledPIDController(7, 0, 0, constraints);
    private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(7, 0, 0, rotationConstraints);


    // Default constructor uses no feedforward
    public DriveToPose(Supplier<Pose2d> reefPoseSupplier, Swerve drivetrain, Supplier<Pose2d> getCurrentPose, boolean noInterpolate, double tolerance) {
        pidControllerOmega.enableContinuousInput(-Math.PI, Math.PI);
        this.reefPoseSupplier = reefPoseSupplier;
        this.drivetrain = drivetrain;
        this.robotPoseSupplier = getCurrentPose;
        addRequirements(drivetrain);
        this.noInterpolate = noInterpolate;
        this.tolerance = tolerance;
    }

    public DriveToPose(Supplier<Pose2d> reefPoseSupplier, Swerve drivetrain) {
        this(reefPoseSupplier, drivetrain, drivetrain::getPose, false, 1);
    }

    public DriveToPose(Supplier<Pose2d> reefPoseSupplier, Swerve drivetrain, boolean noInterpolate) {
        this(reefPoseSupplier, drivetrain, drivetrain::getPose, noInterpolate, 1);
    }


    public DriveToPose(Supplier<Pose2d> reefPoseSupplier, Swerve drivetrain, boolean noInterpolate, double tolerance) {
        this(reefPoseSupplier, drivetrain, drivetrain::getPose, noInterpolate, tolerance);
    }


    @Override
    public void initialize() {
        this.reefPose = reefPoseSupplier.get();
        Pose2d currentPose = drivetrain.getPose();

        pidControllerX.reset(currentPose.getX());
        pidControllerY.reset(currentPose.getY());
        pidControllerOmega.reset(currentPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        // Determine the target pose using your drive target logic.
        Pose2d targetPose = noInterpolate ? reefPose : getDriveTarget(currentPose, reefPose);
        LogUtil.recordPose2d("DriveToPose/Ghost", targetPose);
        LogUtil.recordPose2d("DriveToPose/Reef", reefPose);

        // Calculate feedback speeds from PID controllers.
        double xSpeed = pidControllerX.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = pidControllerY.calculate(currentPose.getY(), targetPose.getY());
        double thetaSpeed = pidControllerOmega.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        // Convert the computed speeds to field-relative chassis speeds.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, thetaSpeed, currentPose.getRotation()
        );
        drivetrain.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        // Finish when both translation and rotation are within tolerances.
        SmartDashboard.putNumber("DriveToPose/TranslationError", drivetrain.getPose().getTranslation().getDistance(reefPose.getTranslation()));
        SmartDashboard.putNumber("DriveToPose/HeadingError", Math.abs(drivetrain.getPose().getRotation().minus(reefPose.getRotation()).getDegrees()));
        SmartDashboard.putBoolean("DriveToPose/TranslationAligned", drivetrain.getPose().getTranslation().getDistance(reefPose.getTranslation()) <= 0.015);
        SmartDashboard.putBoolean("DriveToPose/HeadingAligned", Math.abs(drivetrain.getPose().getRotation().minus(reefPose.getRotation()).getDegrees()) <= 1);
        return drivetrain.getPose().getTranslation().getDistance(reefPose.getTranslation()) <= 0.015 * tolerance &&
                Math.abs(drivetrain.getPose().getRotation().minus(reefPose.getRotation()).getDegrees()) <= 1 * tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    private static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
        Translation2d offset = robot.relativeTo(goal).getTranslation();
        double yDistance = Math.abs(offset.getY());
        double xDistance = Math.abs(offset.getX());
        double shiftXT = MathUtil.clamp((yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)), 0.0, 1.0);
        double shiftYT = MathUtil.clamp(offset.getX() / Reef.faceLength, 0.0, 1.0);

        return goal.transformBy(GeomUtil.toTransform2d(-shiftXT * 1.5, Math.copySign(shiftYT * 1.5 * 0.8, offset.getY())));
    }
}