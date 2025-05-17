package com.team9470.subsystems.vision;

import com.team9470.Robot;
import com.team9470.subsystems.Swerve;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.List;

import static com.team9470.Constants.VisionConstants.FRONT_LEFT_CAMERA_OFFSET;
import static com.team9470.Constants.VisionConstants.FRONT_RIGHT_CAMERA_OFFSET;

public class Vision extends SubsystemBase {
    private static Vision instance;

    private final VisionDevice frontL = new VisionDevice("OV2311-L", FRONT_LEFT_CAMERA_OFFSET);
    private final VisionDevice frontR = new VisionDevice("OV2311-R", FRONT_RIGHT_CAMERA_OFFSET);
    //private final VisionDevice back = new VisionDevice("back", VisionConstants.BACK_CAMERA_OFFSET);
    private boolean visionDisabled = false;

    private final List<VisionDevice> devices = List.of(frontL, frontR);

    // ----------------- SIMULATION -----------------
    private PhotonCameraSim leftCameraSim;
    private PhotonCameraSim rightCameraSim;
    private VisionSystemSim visionSim;

    public Vision(){
        if(Robot.isSimulation()){
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var leftCameraProp = new SimCameraProperties();
            leftCameraProp.setCalibration(1600, 1200, Rotation2d.fromDegrees(90));
            leftCameraProp.setCalibError(0.4, 0.10);
            leftCameraProp.setFPS(25);
            leftCameraProp.setAvgLatencyMs(50);
            leftCameraProp.setLatencyStdDevMs(15);

            var rightCameraProp = new SimCameraProperties();
            rightCameraProp.setCalibration(1600, 1200, Rotation2d.fromDegrees(95.38));
            rightCameraProp.setCalibError(1.1, 0.10);
            rightCameraProp.setFPS(25);
            rightCameraProp.setAvgLatencyMs(50);
            rightCameraProp.setLatencyStdDevMs(15);

            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            leftCameraSim = new PhotonCameraSim(frontL.returnCam(), leftCameraProp);
            rightCameraSim = new PhotonCameraSim(frontR.returnCam(), rightCameraProp);

            leftCameraSim.enableDrawWireframe(true);
            rightCameraSim.enableDrawWireframe(true);

            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(leftCameraSim, FRONT_LEFT_CAMERA_OFFSET);
            visionSim.addCamera(rightCameraSim, FRONT_RIGHT_CAMERA_OFFSET);

        }
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    int count = 0;
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vision/heartbeat", count);
        count++;
        devices.forEach(visionDevice -> visionDevice.updatePosition(Swerve.getInstance()));
    }





    @Override
    public void simulationPeriodic() {
        visionSim.update(Swerve.getInstance().getPose());
    }

    public boolean isFullyConnected(){
        return devices.stream().allMatch(VisionDevice::isConnected);
    }

    public boolean isVisionDisabled() {
        return visionDisabled;
    }

    public void setVisionDisabled(boolean visionDisabled) {
        this.visionDisabled = visionDisabled;
    }
}