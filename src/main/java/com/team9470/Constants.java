package com.team9470;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public final class Constants {
    public static class Global {
        // we want apriltags! see `FieldConstants.java`
        public static final boolean disableHAL = false;
    }

    // TODO: change camera constants
    public static class VisionConstants {
        public static final Transform3d FRONT_LEFT_CAMERA_OFFSET = new Transform3d(Units.Inches.of(-12.498394), Units.Inches.of(+12.145856), Units.Inches.of(+7.715021),
                new Rotation3d(-0.12384803489944651, -0.3272010156831353, 0.3695356336033198));
        public static final Transform3d FRONT_RIGHT_CAMERA_OFFSET = new Transform3d(Units.Inches.of(+12.498394), Units.Inches.of(+12.145856), Units.Inches.of(+7.715021),
                new Rotation3d(0.12384803489944651, -0.3272010156831353, -0.3695356336033198));


    }

    public static final class ElevatorConstants {
        // Physical geometry
        // For example: 22 * 0.25 inches * 2 = 11 inches per rotation
        // Convert to meters for internal usage
        public static final Distance DIST_PER_ROTATION =
                Units.Inches.of(22 * 0.25).times(2);
        public static final double rotationsPerMeter = 1.0 / DIST_PER_ROTATION.in(Meters);
        public static final double GEAR_RATIO = 6.0;
        public static final double MASS = 9.072;
        public static final double DRUM_RADIUS = DIST_PER_ROTATION.in(Meter) / (2 * Math.PI);

        // Gains
        public static final double kP = 7;
        public static final double kG = 0.38;
        public static final double kV = 0.85;
        public static final double kA = 0.01;
        // etc...

        // Motion config
        public static final LinearVelocity CRUISE_VELOCITY = Units.MetersPerSecond.of(2.0);
        public static final LinearAcceleration ACCELERATION = MetersPerSecondPerSecond.of(10.0);
        public static final double JERK = 0;

        // Homing
        public static final Voltage HOMING_OUTPUT = Units.Volts.of(-2.0);
        public static final LinearVelocity HOMING_MAX_VELOCITY = Units.MetersPerSecond.of(0.1);
        public static final Distance HOMING_ZONE = Meters.of(0.1);
        public static final Time HOMING_TIMEOUT = Units.Seconds.of(0.5);

        // Current limits
        public static final double STALL_CURRENT = 40; // example

        public static final Distance HOME_POSITION = Meters.of(0);
        public static final Distance L1 = Meters.of(0.2);
        public static final Distance L2 = Meters.of(.4);
        public static final Distance L3 = Meters.of(.7);
        public static final Distance L4 = Meters.of(1.37);
        public static final Distance INTAKE = Meters.of(0);


        public static TalonFXConfiguration ElevatorFXConfig(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(MetersPerSecond) * rotationsPerMeter;
            SmartDashboard.putNumber("Elevator/CruiseVelocity", config.MotionMagic.MotionMagicCruiseVelocity);
            config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(MetersPerSecondPerSecond) * rotationsPerMeter;
            config.MotionMagic.MotionMagicJerk = JERK;
            config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            config.Slot0.kV = kV;
            config.Slot0.kA = kA;
            config.Slot0.kP = kP;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kG = kG;
            config.Slot0.kS = 0.0;
            config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = STALL_CURRENT;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            return config;
        }

        public static TalonFXConfiguration ElevatorFXConfigFollower(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = STALL_CURRENT;
            return config;
        }
    }

    public static final class AlgaeConstants {
        public static final double KS = 0;
        public static final double KG = 0;
        public static final double KV = 0;
        public static final Angle CORAL_THRESHOLD = Degrees.of(-26); // Angle in degrees
        public static final Current ALGAE_IN_THRESHOLD = Amps.of(5); // Current
        public static final Angle ANGLE_UP = Degrees.of(-30);
        public static final Angle STOW_ANGLE = Degrees.of(-100);

        public static final double CRUISE_VELOCITY = 10;
        public static final double ACCELERATION = 20;
        public static final double JERK = 0;

        public static final Current HOMING_THRESHOLD = Amps.of(5); // Current
        public static final Voltage HOMING_OUTPUT = Volts.of(3);
        public static final Time HOMING_TIMEOUT = Seconds.of(0.5);
        public static final Angle HOMING_ANGLE = Degrees.of(-95);

        public static final Voltage INTAKE_OUTPUT = Volts.of(4);
        public static final Voltage HOLDING_OUTPUT = Volts.of(1);

        public static final double GEAR_RATIO = 4.2;

        public static final double STALL_CURRENT = 40; // Amps

        // sim stuff
        public static final Mass ARM_MASS = Kilogram.of(6.252); // kg
        public static final Distance ARM_LENGTH = Meter.of(.14); // m
        public static final Angle MIN_ANGLE = HOMING_ANGLE; // deg
        public static final Angle MAX_ANGLE = Degrees.of(90); // deg


        public static TalonFXConfiguration getPivotConfig(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
            config.MotionMagic.MotionMagicAcceleration = ACCELERATION;
            config.MotionMagic.MotionMagicJerk = JERK;
            config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
            config.Slot0.kV = 0.48;
            config.Slot0.kA = 0.08;
            config.Slot0.kP = 15;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kG = 0.8;
            config.Slot0.kS = 0.2;
            config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = STALL_CURRENT;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            return config;
        }

        public static TalonFXConfiguration getRollerConfig(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            return config;
        }
    }

    public static final class CoralConstants {
        public static final Voltage TAKE_IN_SPEED = Volts.of(3);
        public static final Voltage COAST_SPEED = Volts.of(2);
        public static final Voltage FUNNEL_SPEED = Volts.of(-3);
        public static final Voltage HOLD_SPEED = Volts.of(-.5);
        public static final double BREAK_TIMEOUT = .1;
    }

    public static final class DriverAssistConstants { // TODO: maybe you can use hexagon math to calculate these? also, there are TWELVE positions per reef, not six. im also sure theres a flipper method in fieldconstants, but you may have to check.
        // public static final Pose2d[] BLUE_REEF_POSITIONS = { // {x (m), y (m), angle (rad)}
        //     new Pose2d(3.7454309463500977, 5.406795501708984, new Rotation2d(-1.0584074157409784)),
        //     new Pose2d(2.9004666805267334, 4.025999546051025, new Rotation2d(0)),
        //     new Pose2d(3.7042129039764404, 2.6452038288116455, new Rotation2d(1.0303770533621297)),
        //     new Pose2d(5.270488739013672, 2.645203113555908, new Rotation2d(2.1375260206777438)),
        //     new Pose2d(6.094844341278076, 4.025998592376709, new Rotation2d(3.141592653589793)),
        //     new Pose2d(5.249879837036133, 5.40679407119751, new Rotation2d(-2.095592098445004)),
        // };

        // public static final Pose2d[] RED_REEF_POSITIONS = { // {x (m), y (m), angle (rad)}
        //     new Pose2d(12.339337348937988, 5.406795501708984, new Rotation2d(-1.0584074157409784)),
        //     new Pose2d(11.514982223510742, 4.025999546051025, new Rotation2d(0)),
        //     new Pose2d(12.29811954498291, 2.6452038288116455, new Rotation2d(1.0303770533621297)),
        //     new Pose2d(13.864395141601562, 2.645203113555908, new Rotation2d(2.1375260206777438)),
        //     new Pose2d(14.647533416748047, 4.025998592376709, new Rotation2d(3.141592653589793)),
        //     new Pose2d(13.823177337646484, 5.40679407119751, new Rotation2d(-2.095592098445004)),
        // };

        public static final double centerX = 4.47675;
        public static final double centerY = 4.0259;
        public static final double radius = 1.58115;
        public static final double pipeDistance = 0.1651;
        public static final double fieldLength = 17.548225;
        
        public static Pose2d[] getReefPositions(DriverStation.Alliance alliance){
            Pose2d[] REEF_POSITIONS = new Pose2d[12];
            for (int i = 0; i < 6; i++) {
                double angle1 = Math.PI / 6 + Math.PI / 3 * i;
                double angle2 = Math.PI / 6 + Math.PI / 3 * (i + 1);
                
                double x1 = centerX + radius * Math.cos(angle1);
                double y1 = centerY + radius * Math.sin(angle1);
                double x2 = centerX + radius * Math.cos(angle2);
                double y2 = centerY + radius * Math.sin(angle2);
                
                // Compute the midpoint of the side
                double midX = (x1 + x2) / 2;
                double midY = (y1 + y2) / 2;

                
                // Compute the angle to face away from the hexagon center
                if(alliance == DriverStation.Alliance.Blue){
                    double faceAngle = Math.atan2(midY - centerY, midX - centerX) + Math.PI;
                    
                    REEF_POSITIONS[2*i] = new Pose2d(midX - pipeDistance * Math.sin(Math.PI / 3 * (i-2)), midY + pipeDistance * Math.cos(Math.PI / 3 * (i-2)), new Rotation2d(faceAngle));
                    REEF_POSITIONS[2*i+1] = new Pose2d(midX + pipeDistance * Math.sin(Math.PI / 3 * (i-2)), midY - pipeDistance * Math.cos(Math.PI / 3 * (i-2)), new Rotation2d(faceAngle));
                }
                else{
                    double faceAngle = -Math.atan2(midY - centerY, midX - centerX);
                    
                    REEF_POSITIONS[2*i] = new Pose2d(fieldLength - midX + pipeDistance * Math.sin(Math.PI / 3 * (i-2)), midY + pipeDistance * Math.cos(Math.PI / 3 * (i-2)), new Rotation2d(faceAngle));
                    REEF_POSITIONS[2*i+1] = new Pose2d(fieldLength - midX - pipeDistance * Math.sin(Math.PI / 3 * (i-2)), midY - pipeDistance * Math.cos(Math.PI / 3 * (i-2)), new Rotation2d(faceAngle));
                }
                System.out.println(REEF_POSITIONS[2*i]);
                System.out.println(REEF_POSITIONS[2*i+1]);
            }
            return REEF_POSITIONS;
        }
    }
}