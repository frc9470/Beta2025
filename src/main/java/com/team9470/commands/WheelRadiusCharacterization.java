package com.team9470.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team9470.subsystems.Swerve;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

public class WheelRadiusCharacterization extends Command {
    private static final double characterizationSpeed = .1;
    private static final double driveRadius = Units.inchesToMeters(12.5);
    private static final DoubleSupplier gyroYawRadsSupplier =
            () -> Swerve.getInstance().getPose().getRotation().getRadians();

    @RequiredArgsConstructor
    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final int value;
    }

    private final Swerve drive;
    private final Direction omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double[] startWheelPositions;

    private double currentEffectiveWheelRadius = 0.0;

    public WheelRadiusCharacterization(Swerve drive, Direction omegaDirection) {
        this.drive = drive;
        this.omegaDirection = omegaDirection;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        startWheelPositions = drive.getWheelRadiusCharacterizationPosition();

        omegaLimiter.reset(0);
    }

    SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    @Override
    public void execute() {
        // Run drive at velocity
        drive.setControl(driveRequest.withRotationalRate(
                omegaLimiter.calculate(omegaDirection.value * characterizationSpeed*2*Math.PI)));

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        double averageWheelPosition = 0.0;
        double[] wheelPositiions = drive.getWheelRadiusCharacterizationPosition();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
        SmartDashboard.putNumber("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
        SmartDashboard.putNumber("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
        SmartDashboard.putNumber(
                "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
                Units.metersToInches(currentEffectiveWheelRadius));
    }

    @Override
    public void end(boolean interrupted) {
        drive.setControl(new SwerveRequest.RobotCentric());
        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        } else {
            System.out.println(
                    "Effective Wheel Radius: "
                            + Units.metersToInches(currentEffectiveWheelRadius)
                            + " inches");
        }
    }
}