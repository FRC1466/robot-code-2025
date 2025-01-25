/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 

 package frc.robot.subsystems;

 import static frc.robot.Constants.VisionConstants.*;
 
 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
 import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Pose3d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import edu.wpi.first.wpilibj.DriverStation;
 import edu.wpi.first.wpilibj.DriverStation.Alliance;
 import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
 import frc.robot.Robot;

import java.util.List;
import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonPipelineResult;
 
 public class PhotonCameraWrapper {
    private PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;

    private PhotonCameraSim cameraSim;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private VisionSystemSim visionSim;

    private List<PhotonPipelineResult> cameraResult;
    private double lastEstTimestamp = 0;

     public PhotonCameraWrapper(){
        camera = new PhotonCamera("Global_Shutter_Camera");
        var result = camera.getLatestResult();
        aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
        var kRobotToCam = new Transform3d(Constants.cameraTranslation, Constants.cameraRotation);

       photonEstimator = new PhotonPoseEstimator(
                        aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
       /*  if(result.hasTargets()){
            SmartDashboard.putNumberArray("Best Target Pose", photonEstimator.);
        }

        // boolean hasTargets = result.hasTargets();


     }

     public void setOrigin(){
  
        aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    }
    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public Optional<Pose3d> AprilTagPose(int apriltagNum){
        return aprilTagFieldLayout.getTagPose(apriltagNum);
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update(camera.getLatestResult());
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-3;
       /*  if (Robot.isSimulation()) {
            visionEst.ifPresentOrElse(
                    est ->
                            getSimDebugField()
                                    .getObject("VisionEstimation")
                                    .setPose(est.estimatedPose.toPose2d()),
                    () -> {
                        if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
                    });
        }
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
     

          else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

   
}*/
