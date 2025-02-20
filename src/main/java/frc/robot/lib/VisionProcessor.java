package frc.robot.lib;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**  */
public class VisionProcessor implements IUpdateDashboard{
    
    PhotonCamera m_leftCamera;
    PhotonPoseEstimator m_leftPoseEstimator;

    PhotonCamera m_rightCamera;
    PhotonPoseEstimator m_rightPoseEstimator;

    boolean isTartgetFound = false;

    boolean m_resetYawInitFlag = false;
    AprilTagFieldLayout m_apriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public VisionProcessor(){

        m_leftCamera = new PhotonCamera("LeftArducam");
        m_leftCamera.setPipelineIndex(0);
        m_leftCamera.setDriverMode(false);
        Transform3d m_leftCameraLocation = new Transform3d(new Translation3d(0.2972,0.2667,0.26), new Rotation3d(0,Math.toRadians(-12.5),Math.toRadians(-10)));
        m_leftPoseEstimator = new PhotonPoseEstimator(m_apriltagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_leftCameraLocation);

        
        m_rightCamera = new PhotonCamera("RightCamera");
        m_rightCamera.setPipelineIndex(0);
        m_rightCamera.setDriverMode(false);
        Transform3d m_rightCameraLocation = new Transform3d(new Translation3d(0.2972,-0.2667,0.26), new Rotation3d(0,Math.toRadians(12.5),Math.toRadians(-10)));
        m_rightPoseEstimator = new PhotonPoseEstimator(m_apriltagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_rightCameraLocation);
        g.DASHBOARD.updates.add(this);
        createApriltagLocations();
        
    }
    public Pose2d getRobotPoseForAprilTag(int _id, AprilTagAlignState _apriltagAlignState ){
        Pose2d rtn;
        ApriltagPose pose = g.AprilTagLocations.pose.get(_id);

        switch (_apriltagAlignState) {
            case LEFT:
                rtn = new Pose2d(pose.x_left,pose.y_left , new Rotation2d(Math.toRadians(pose.angle)));
                break;
            case RIGHT:
                rtn = new Pose2d(pose.x_right,pose.y_right , new Rotation2d(Math.toRadians(pose.angle)));
                break;
            case CENTER:
                rtn = new Pose2d(pose.x_center,pose.y_center , new Rotation2d(Math.toRadians(pose.angle)));
                break;
            case NONE:
                 rtn = new Pose2d(pose.x_center,pose.y_center , new Rotation2d(Math.toRadians(pose.angle)));
                break;
            default:
                rtn = new Pose2d(pose.x_center,pose.y_center , new Rotation2d(Math.toRadians(pose.angle)));
                break;
        }
        return rtn;
    }
    /**
     *  Create the x,y locations for the robot with reference to the Blue field.
     *  The angle is the angle the robot should face in reference to the Blue or Red field with respect to zero is away from the alliance wall. CW is negative
     */
    private void createApriltagLocations(){
        double x,y,cx,cy;

        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 0 which does not exist on the map
        g.AprilTagLocations.pose.add(new ApriltagPose(15.9576, 0.6263, 16.9339, 1.3429, 16.4408, 0.9846, -54.011));  // ID 1 red left station. Only care about center at this point
        g.AprilTagLocations.pose.add(new ApriltagPose(16.9339, 6.6872, 15.9476, 7.4039, 16.4408, 7.0456, 54.011));  //  ID 2 red right station. Only care about center at this point
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 3 red processor. Only care about center at this point
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 4
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 5
        x = m_apriltagFieldLayout.getTagPose(6).get().getX();
        y = m_apriltagFieldLayout.getTagPose(6).get().getY();
        cx = x + g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y - g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx - g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx + g.FIELD.TAG_TO_POST_m * 0.866, cy + g.FIELD.TAG_TO_POST_m / 2, cx, cy, -60));  // ID 6
        x = m_apriltagFieldLayout.getTagPose(7).get().getX();
        y = m_apriltagFieldLayout.getTagPose(7).get().getY();
        cx = x + g.ROBOT.centerDistanceToFrontBumper_m;
        cy = y;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx, cy - g.FIELD.TAG_TO_POST_m, cx, cy + g.FIELD.TAG_TO_POST_m, cx, cy, 0));  // ID 7
        x = m_apriltagFieldLayout.getTagPose(8).get().getX();
        y = m_apriltagFieldLayout.getTagPose(8).get().getY();
        cx = x + g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y + g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx + g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx - g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx, cy,60));  // ID 8
        x = m_apriltagFieldLayout.getTagPose(9).get().getX();
        y = m_apriltagFieldLayout.getTagPose(9).get().getY();
        cx = x - g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y + g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx + g.FIELD.TAG_TO_POST_m * 0.866, cy + g.FIELD.TAG_TO_POST_m / 2, cx - g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx, cy, 120));  // ID 9
        x = m_apriltagFieldLayout.getTagPose(10).get().getX();
        y = m_apriltagFieldLayout.getTagPose(10).get().getY();
        cx = x-g.ROBOT.centerDistanceToFrontBumper_m;
        cy = y;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx, cy+g.FIELD.TAG_TO_POST_m, cx, cy - g.FIELD.TAG_TO_POST_m, cx, cy, 180));  // ID 10
        x = m_apriltagFieldLayout.getTagPose(11).get().getX();
        y = m_apriltagFieldLayout.getTagPose(11).get().getY();
        cx = x - g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y - g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx-(g.FIELD.TAG_TO_POST_m*0.866), cy+g.FIELD.TAG_TO_POST_m/2, cx + g.FIELD.TAG_TO_POST_m*0.866,cy - g.FIELD.TAG_TO_POST_m/2, cx, cy, -120));   // ID 11
        x = m_apriltagFieldLayout.getTagPose(12).get().getX();
        y = m_apriltagFieldLayout.getTagPose(12).get().getY();
        g.AprilTagLocations.pose.add(new ApriltagPose(0.6315, 1.3429, 1.5871,0.6445,1.1247,0.9846, 54.011));  // ID 12
        x = m_apriltagFieldLayout.getTagPose(13).get().getX();
        y = m_apriltagFieldLayout.getTagPose(13).get().getY();
        g.AprilTagLocations.pose.add(new ApriltagPose(1.6789, 7.4039, 0.6315, 6.6872, 1.1247, 7.0456, -54.011));  // ID 13
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 14
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 15
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 16
        x = m_apriltagFieldLayout.getTagPose(17).get().getX();
        y = m_apriltagFieldLayout.getTagPose(17).get().getY();
        cx = x - g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y - g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx-(g.FIELD.TAG_TO_POST_m*0.866), cy+g.FIELD.TAG_TO_POST_m/2, cx + g.FIELD.TAG_TO_POST_m*0.866,cy - g.FIELD.TAG_TO_POST_m/2, cx, cy, 60));  // ID 17
        x = m_apriltagFieldLayout.getTagPose(18).get().getX();
        y = m_apriltagFieldLayout.getTagPose(18).get().getY();
        cx = x-g.ROBOT.centerDistanceToFrontBumper_m;
        cy = y;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx, cy+g.FIELD.TAG_TO_POST_m, cx, cy - g.FIELD.TAG_TO_POST_m, cx, cy, 0));  // ID 18
        x = m_apriltagFieldLayout.getTagPose(19).get().getX();
        y = m_apriltagFieldLayout.getTagPose(19).get().getY();
        cx = x - g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y + g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx + g.FIELD.TAG_TO_POST_m * 0.866, cy + g.FIELD.TAG_TO_POST_m / 2, cx - g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx, cy, -60));  // ID 19
        x = m_apriltagFieldLayout.getTagPose(20).get().getX();
        y = m_apriltagFieldLayout.getTagPose(20).get().getY();
        cx = x + g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y + g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx + g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx - g.FIELD.TAG_TO_POST_m * 0.866, cy + g.FIELD.TAG_TO_POST_m / 2, cx, cy,-120));  // ID 20
        x = m_apriltagFieldLayout.getTagPose(21).get().getX();
        y = m_apriltagFieldLayout.getTagPose(21).get().getY();
        cx = x + g.ROBOT.centerDistanceToFrontBumper_m;
        cy = y;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx, cy - g.FIELD.TAG_TO_POST_m, cx, cy + g.FIELD.TAG_TO_POST_m, cx, cy, 180));  // ID 21
        x = m_apriltagFieldLayout.getTagPose(22).get().getX();
        y = m_apriltagFieldLayout.getTagPose(22).get().getY();
        cx = x + g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y - g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx - g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx + g.FIELD.TAG_TO_POST_m * 0.866, cy + g.FIELD.TAG_TO_POST_m / 2, cx, cy, 120)); // ID 22
    }
    
    public PoseEstimateStatus calculatePose(PhotonCamera _camera, PhotonPoseEstimator _poseEstimtor) {
        double ambiguity = 0;
        TagFoundState tagState = TagFoundState.EMPTY;
        List<PhotonPipelineResult> results = _camera.getAllUnreadResults(); // Get all results from the apriltag pipeline.
        if (!results.isEmpty()) { // If there are no results from the pipeline the results is empty. This happens 2 times. 1. No tag found, 2. Pipeline flushed to often with no new results
            for (PhotonPipelineResult photonPipelineResult : results) {
                if(photonPipelineResult.hasTargets()){
                    List<PhotonTrackedTarget> targets = photonPipelineResult.getTargets();
                    if(!targets.isEmpty()){
                        for (PhotonTrackedTarget target : targets) {
                            ambiguity = target.poseAmbiguity;

                            if(ambiguity >= 0 && ambiguity < g.VISION.ambiguitySetPoint){
                                if(tagState != TagFoundState.TARGET_ID_FOUND){
                                    tagState = TagFoundState.TAG_FOUND;
                                }
                                Optional<EstimatedRobotPose> estimatedRobotPose = _poseEstimtor.update(photonPipelineResult);
                                if(estimatedRobotPose.isPresent()){
                                    g.ROBOT.drive.addVisionMeasurement(estimatedRobotPose.get().estimatedPose.toPose2d(), estimatedRobotPose.get().timestampSeconds);
                                    g.VISION.pose2d = estimatedRobotPose.get().estimatedPose.toPose2d();
                                }
                                if(target.getFiducialId() == g.VISION.aprilTagRequestedID){
                                    tagState = TagFoundState.TARGET_ID_FOUND;
                                }
                            }
                        }
                    }
                }
            }
        }
        g.VISION.tagState = tagState;
        return new PoseEstimateStatus(g.VISION.tagState, ambiguity);
    }
    // public double getTargetIDAngle(double _id){
    //     return getTargetIDAngle(m_leftCamera, m_leftPoseEstimator, 20);
    // }
    // public double getTargetIDAngle(PhotonCamera _camera, PhotonPoseEstimator _poseEstimtor, int _id){
    //     double ambiguity = 0;
    //     List<PhotonPipelineResult> results = _camera.getAllUnreadResults(); // Get all results from the apriltag pipeline.
    //     if (!results.isEmpty()) { // If there are no results from the pipeline the results is empty. This happens 2 times. 1. No tag found, 2. Pipeline flushed to often with no new results
    //         for (PhotonPipelineResult photonPipelineResult : results) {
    //             if(photonPipelineResult.hasTargets()){
    //                 List<PhotonTrackedTarget> targets = photonPipelineResult.getTargets();
    //                 if(!targets.isEmpty()){
    //                     for (PhotonTrackedTarget target : targets) {
    //                         ambiguity = target.poseAmbiguity;
    //                         if(ambiguity >= 0 && ambiguity < g.VISION.ambiguitySetPoint){
    //                             if(target.getFiducialId() == _id){
    //                                 g.VISION.initTargetIDAngle = target.yaw;
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }


    //     return  g.VISION.initTargetIDAngle;
    // }
    // public void setOdometry(StartLocation _start) {
    //     switch (_start) {
    //       case LEFT:
    //       m_rightPoseEstimator.setLastPose(g.ROBOT.POSE_START_LEFT);
    //       m_leftPoseEstimator.setLastPose(g.ROBOT.POSE_START_LEFT);
    //         break;
    //       case RIGHT:
    //       m_rightPoseEstimator.setLastPose(g.ROBOT.POSE_START_RIGHT);
    //       m_leftPoseEstimator.setLastPose(g.ROBOT.POSE_START_RIGHT);
    //         break;
    //       case CENTER:
    //       m_rightPoseEstimator.setLastPose(g.ROBOT.POSE_START_CENTER);
    //       m_leftPoseEstimator.setLastPose(g.ROBOT.POSE_START_CENTER);
    //         break;
    //       case ZERO:
    //       m_rightPoseEstimator.setLastPose(g.ROBOT.POSE_START_ZERO);
    //       m_rightPoseEstimator.setLastPose(g.ROBOT.POSE_START_ZERO);
    //         break;
    //       default:
    //       m_rightPoseEstimator.setLastPose(g.ROBOT.POSE_START_LEFT);
    //       m_leftPoseEstimator.setLastPose(g.ROBOT.POSE_START_LEFT);
    //         break;
    //     }
    //   }
    /*
     * Even if the tag is seen a lot of results are empty. 
     * Options:
     * 1. deal iwth the tagfound flickering.
     * 2. Ignore empty for a certain amount of time. 
     * 3. Figure out a way to limit it in backend
     * 4. Ignore is tag found as an indicator, still set it and the autopose will happen eventually
     * 5. We can assume empty results are where there are no values left in FIFO because we are calling it at a 5ms rate.
     *    Therefore we can ignore the Empty results because results are a camera capture not just a apriltag. Even 
     *    if it is a april tag we don't care about empty. But if it is empty because no tags we have to deal with something else.
     * 6. Doing vision at 5ms means it is asking for results at a 200hz rate. The camera only does at most 50FPS. Therefore this should not be done at 200Hz.
     */
    
    public void calculatePose(){
        PoseEstimateStatus leftCamState = null;
        PoseEstimateStatus rightCamState = null;

        if (DriverStation.getAlliance().isPresent()) {
            // Calculate the pose for the left camera
            g.VISION.aprilTagRequestedID = getAprilTagID(g.ROBOT.alignmentState, DriverStation.getAlliance().get());
            if (m_leftCamera.isConnected()) {
                leftCamState = calculatePose(m_leftCamera, m_leftPoseEstimator);
                g.VISION.leftTargetAmbiguity = leftCamState.getAmbiguity();
            }
            // Calculate the pose for the right camera
            if (m_rightCamera.isConnected()) {
                rightCamState = calculatePose(m_rightCamera, m_rightPoseEstimator);
                g.VISION.rightTargetAmbiguity = rightCamState.getAmbiguity();
            }
            if (m_rightCamera.isConnected() && m_leftCamera.isConnected()) {
                double angle = Math.abs(g.VISION.pose2d.getRotation().getDegrees());
                if (!m_resetYawInitFlag && angle > 5.0) {
                    if (g.VISION.leftTargetAmbiguity >= 0.0 && g.VISION.leftTargetAmbiguity < g.VISION.ambiguitySetPoint &&
                        g.VISION.rightTargetAmbiguity >= 0.0 && g.VISION.rightTargetAmbiguity < g.VISION.ambiguitySetPoint) {
                        g.ROBOT.drive.resetYaw(g.VISION.pose2d.getRotation().getDegrees());
                        m_resetYawInitFlag = true;
                    }
                }
            }

            if (leftCamState != null && rightCamState != null) {
                if (leftCamState.getState() == TagFoundState.TARGET_ID_FOUND || rightCamState.m_state == TagFoundState.TARGET_ID_FOUND) {
                    g.VISION.isTargetAprilTagFound = true;
             
                } else if (leftCamState.m_state == TagFoundState.EMPTY && rightCamState.getState() == TagFoundState.EMPTY) {
                    g.VISION.isTargetAprilTagFound = false;
                }
            }
        }
    }
    /* Notes for trusting vision pose.
     * Things we know:
     * 1. Wheel slip will happen
     * 2. Drive wheels will become inaccurate.
     * 3. Drive is better if no tags are visible
     * 4. Vision is very accurate
     * 5. Driving to target pose from short distance should be done without rotation of modules
     *    No slippage, 
     * 
     */
   
    /**
     * Based on the apriltag alignment, which is LEFT,CENTER,RIGHT or NONE.
     * Also if the Apriltag is found by vision. The vision only reports true if the RobotAlignment has been set
     * and the ID of that tag has been found.
     * @return
     */
    public boolean getIsAutoAprilTagActive(){
        if (g.VISION.aprilTagAlignState != AprilTagAlignState.NONE 
            && g.VISION.tagState == TagFoundState.TARGET_ID_FOUND
            && g.DRIVETRAIN.isAutoDriveEnabled) {
            return true;
        }
        if (g.VISION.aprilTagAlignState != AprilTagAlignState.NONE && g.DRIVETRAIN.isAutoDriveEnabled)
            if (g.VISION.aprilTagRequestedID == 12 
                || g.VISION.aprilTagRequestedID == 13 
                || g.VISION.aprilTagRequestedID == 1 
                || g.VISION.aprilTagRequestedID == 2) {
                return true;
            }
        return false;
    }
    public int getAprilTagID(RobotAlignStates _alignState, Alliance _alliance) {
        int rtn = 0; // 0 is an invalid ID
        switch (_alignState) {
            case BACK:
                rtn = _alliance == Alliance.Blue ? 21 : 10;
                break;
            case BACK_LEFT:
                rtn = _alliance == Alliance.Blue ? 20 : 11;
                break;
            case BACK_RIGHT:
                rtn = _alliance == Alliance.Blue ? 22 : 9;
                break;
            case FRONT:
                rtn = _alliance == Alliance.Blue ? 18 : 7;
                break;
            case FRONT_LEFT:
                rtn = _alliance == Alliance.Blue ? 19 : 6;
                break;
            case FRONT_RIGHT:
                rtn = _alliance == Alliance.Blue ? 17 : 8;
                break;
            case LEFT:
                rtn = _alliance == Alliance.Blue ? 3 : 16;
                break;
            case RIGHT:
                rtn = _alliance == Alliance.Blue ? 16 : 3;
                break;
            case STATION_LEFT:
                rtn = _alliance == Alliance.Blue ? 13 : 1;
                break;
            case STATION_RIGHT:
                rtn = _alliance == Alliance.Blue ? 12 : 2;
                break;
            case UNKNOWN:
                rtn = 0;
                break;
            default:
                rtn = 0;
                break;
        }
        return rtn;

    }

    @Override
    public void updateDashboard() {
        g.VISION.field2d.setRobotPose(g.VISION.pose2d);
        SmartDashboard.putNumber("Vision/Apriltag Requested ID", g.VISION.aprilTagRequestedID);
        //SmartDashboard.putNumber("Vision/Empty Tag Cnt", m_tagEmptyCnt);
        SmartDashboard.putNumber("Vision/Pose Angle", g.VISION.pose2d.getRotation().getDegrees());
        //SmartDashboard.putNumber("Vision/AprilTag Requested Pose X", g.VISION.aprilTagRequestedPose.getX());
        //SmartDashboard.putNumber("Vision/AprilTag Requested Pose Y", g.VISION.aprilTagRequestedPose.getY());
        SmartDashboard.putString("Vision/Apriltag AlignState", g.VISION.aprilTagAlignState.toString());
        SmartDashboard.putString("Vision/AprilTagFoundState", g.VISION.tagState.toString());
        SmartDashboard.putData("Vision/Vision Field2d", g.VISION.field2d);
        //SmartDashboard.putNumber("Vision/InitTargetAngle",  g.VISION.initTargetIDAngle);
        //SmartDashboard.putNumber("Vision/Pose Vision X", g.VISION.pose2d.getX());
        //SmartDashboard.putNumber("Vision/Pose Vision Y", g.VISION.pose2d.getY());

    }
}
