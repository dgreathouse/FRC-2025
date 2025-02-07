package frc.robot.lib;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**  */
public class VisionProcessor implements IUpdateDashboard{
    
    VisionPoseEstimatorThread m_poseEstimatorThread;
    PhotonCamera m_leftCamera;
    PhotonPoseEstimator m_leftPoseEstimator;

    PhotonCamera m_rightCamera;
    PhotonPoseEstimator m_rightPoseEstimator;

    PhotonCamera m_backCamera;
    PhotonPoseEstimator m_backPoseEstimator;
    
    boolean isTartgetFound = false;
     
    AprilTagFieldLayout m_apriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public VisionProcessor(){
        m_leftCamera = new PhotonCamera("leftArducam");
        m_leftCamera.setPipelineIndex(0);
        m_leftCamera.setDriverMode(false);
        // TODO: update cameral location on robot. x forward, y left, z up
        Transform3d m_leftCameraLocation = new Transform3d(new Translation3d(0.254,0.254,0.292), new Rotation3d(0,0,0));
        m_leftPoseEstimator = new PhotonPoseEstimator(m_apriltagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_leftCameraLocation);

        m_rightCamera = new PhotonCamera("rightArducam");
        m_rightCamera.setPipelineIndex(0);
        m_rightCamera.setDriverMode(false);
        // TODO: update cameral location on robot. x forward, y left, z up
        Transform3d m_rightCameraLocation = new Transform3d(new Translation3d(0.2254,-0.254,0.292), new Rotation3d(0,0,0));
        m_rightPoseEstimator = new PhotonPoseEstimator(m_apriltagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_rightCameraLocation);

        
        m_backCamera = new PhotonCamera("backArducam");
        m_backCamera.setPipelineIndex(0);
        m_backCamera.setDriverMode(false);
        // TODO: update cameral location on robot. x forward, y left, z up
        Transform3d m_backCameraLocation = new Transform3d(new Translation3d(-0.2254,0,0.292), new Rotation3d(0,0,Math.toRadians(180)));
        m_backPoseEstimator = new PhotonPoseEstimator(m_apriltagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_backCameraLocation);

        g.DASHBOARD.updates.add(this);
        createApriltagLocations();

        m_poseEstimatorThread = new VisionPoseEstimatorThread();
        m_poseEstimatorThread.start();

        
    }
    public Pose2d getRobotLocationToAprilTag(int _id, AprilTagAlignState _apriltagAlignState ){
        Pose2d rtn;
        ApriltagPose pose = g.AprilTagLocations.pose.get(_id);

        switch (_apriltagAlignState) {
            case LEFT:
                rtn = new Pose2d(pose.x_left,pose.y_left , new Rotation2d());
                break;
            case RIGHT:
                rtn = new Pose2d(pose.x_right,pose.y_right , new Rotation2d());
                break;
            case CENTER:
                rtn = new Pose2d(pose.x_center,pose.y_center , new Rotation2d());
                break;
            case NONE:
                 rtn = new Pose2d(pose.x_center,pose.y_center , new Rotation2d());
                break;
            default:
                rtn = new Pose2d(pose.x_center,pose.y_center , new Rotation2d());
                break;
        }
        return rtn;
    }
    private void createApriltagLocations(){
        double x,y,cx,cy;
        // Every triangle is a 30:60:90. This follows the 1:sqrt(3):2 rule. The short end is 1, hypotenuse is 2 times the short end and the other side is sqrt(3) times the short end.
        // Example: Hypotenuse is 164.28mm, therfore the short end is 1/2 the hypontenuse or 82.14mm. The other side is sqrt(3)=1.723 or 1.732*82.14 = 142.27
        // Every coral post is 163.28mm from the center of the apriltag.
        // These numbers are to represent the pose our robot must be at to interact correctly with the field object like the coral posts. 
        // We want to use these numbers directly as the pose our robot must go to. This calculation being done now just stores the pose of the apriltag on the field, not our needed robot pose.
        // A pose for our robot represents where the center of our robot should go to. Therefore we will need to calculate that and add that in through a calculation or change these numbers.
        // The nose of our robot is 535mm from the center of the robot. The nose will always be pointing at the coral posts.
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 0 which does not exist on the map
        g.AprilTagLocations.pose.add(new ApriltagPose(15.9576, 0.6263, 16.9339, 1.3429, 16.4408, 0.9846, 0));  // ID 1 red left station. Only care about center at this point
        g.AprilTagLocations.pose.add(new ApriltagPose(16.9339, 6.6872, 15.9476, 7.4039, 16.4408, 7.0456, 0));  // ID 2 red right station. Only care about center at this point
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 3 red processor. Only care about center at this point
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 4
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 5
        x = m_apriltagFieldLayout.getTagPose(6).get().getX();
        y = m_apriltagFieldLayout.getTagPose(6).get().getY();
        cx = x + g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y - g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx - g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx + g.FIELD.TAG_TO_POST_m * 0.866, cy + g.FIELD.TAG_TO_POST_m / 2, cx, cy, 0));  // ID 6
        x = m_apriltagFieldLayout.getTagPose(7).get().getX();
        y = m_apriltagFieldLayout.getTagPose(7).get().getY();
        cx = x + g.ROBOT.centerDistanceToFrontBumper_m;
        cy = y;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx, cy - g.FIELD.TAG_TO_POST_m, cx, cy + g.FIELD.TAG_TO_POST_m, cx, cy, 0));  // ID 7
        x = m_apriltagFieldLayout.getTagPose(8).get().getX();
        y = m_apriltagFieldLayout.getTagPose(8).get().getY();
        cx = x + g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y + g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx + g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx - g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx, cy, 0));  // ID 8
        x = m_apriltagFieldLayout.getTagPose(9).get().getX();
        y = m_apriltagFieldLayout.getTagPose(9).get().getY();
        cx = x - g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y + g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx + g.FIELD.TAG_TO_POST_m * 0.866, cy + g.FIELD.TAG_TO_POST_m / 2, cx - g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx, cy, 0));  // ID 9
        x = m_apriltagFieldLayout.getTagPose(10).get().getX();
        y = m_apriltagFieldLayout.getTagPose(10).get().getY();
        cx = x-g.ROBOT.centerDistanceToFrontBumper_m;
        cy = y;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx, cy+g.FIELD.TAG_TO_POST_m, cx, cy - g.FIELD.TAG_TO_POST_m, cx, cy, 180));  // ID 10
        x = m_apriltagFieldLayout.getTagPose(11).get().getX();
        y = m_apriltagFieldLayout.getTagPose(11).get().getY();
        cx = x - g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y - g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx-(g.FIELD.TAG_TO_POST_m*0.866), cy+g.FIELD.TAG_TO_POST_m/2, cx + g.FIELD.TAG_TO_POST_m*0.866,cy - g.FIELD.TAG_TO_POST_m/2, cx, cy, 0));   // ID 11
        x = m_apriltagFieldLayout.getTagPose(12).get().getX();
        y = m_apriltagFieldLayout.getTagPose(12).get().getY();
        g.AprilTagLocations.pose.add(new ApriltagPose(0.6315, 1.3429, 1.6789,0.6263,1.1247,0.9846,0));  // ID 12
        x = m_apriltagFieldLayout.getTagPose(13).get().getX();
        y = m_apriltagFieldLayout.getTagPose(13).get().getY();
        g.AprilTagLocations.pose.add(new ApriltagPose(1.6789, 7.4039, 0.6315, 6.6872, 1.1247, 7.0456, 0));  // ID 13
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 14
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 15
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 16
        x = m_apriltagFieldLayout.getTagPose(17).get().getX();
        y = m_apriltagFieldLayout.getTagPose(17).get().getY();
        cx = x - g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y - g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx-(g.FIELD.TAG_TO_POST_m*0.866), cy+g.FIELD.TAG_TO_POST_m/2, cx + g.FIELD.TAG_TO_POST_m*0.866,cy - g.FIELD.TAG_TO_POST_m/2, cx, cy, 0));  // ID 17
        x = m_apriltagFieldLayout.getTagPose(18).get().getX();
        y = m_apriltagFieldLayout.getTagPose(18).get().getY();
        cx = x-g.ROBOT.centerDistanceToFrontBumper_m;
        cy = y;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx, cy+g.FIELD.TAG_TO_POST_m, cx, cy - g.FIELD.TAG_TO_POST_m, cx, cy, 0));  // ID 18
        x = m_apriltagFieldLayout.getTagPose(19).get().getX();
        y = m_apriltagFieldLayout.getTagPose(19).get().getY();
        cx = x - g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y + g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx + g.FIELD.TAG_TO_POST_m * 0.866, cy + g.FIELD.TAG_TO_POST_m / 2, cx - g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx, cy, 0));  // ID 19
        x = m_apriltagFieldLayout.getTagPose(20).get().getX();
        y = m_apriltagFieldLayout.getTagPose(20).get().getY();
        cx = x + g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y + g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx + g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx - g.FIELD.TAG_TO_POST_m * 0.866, cy + g.FIELD.TAG_TO_POST_m / 2, cx, cy, 0));  // ID 20
        x = m_apriltagFieldLayout.getTagPose(21).get().getX();
        y = m_apriltagFieldLayout.getTagPose(21).get().getY();
        cx = x + g.ROBOT.centerDistanceToFrontBumper_m;
        cy = y;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx, cy - g.FIELD.TAG_TO_POST_m, cx, cy + g.FIELD.TAG_TO_POST_m, cx, cy, 0));  // ID 21
        x = m_apriltagFieldLayout.getTagPose(22).get().getX();
        y = m_apriltagFieldLayout.getTagPose(22).get().getY();
        cx = x + g.ROBOT.centerDistanceToFrontBumper_m/2;
        cy = y - g.ROBOT.centerDistanceToFrontBumper_m * 0.866;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx - g.FIELD.TAG_TO_POST_m * 0.866, cy - g.FIELD.TAG_TO_POST_m / 2, cx + g.FIELD.TAG_TO_POST_m * 0.866, cy + g.FIELD.TAG_TO_POST_m / 2, cx, cy, 0)); // ID 22
    }
    double emptyCnt = 0;
    public TagFoundState calculatePose(PhotonCamera _camera, PhotonPoseEstimator _poseEstimtor) {
        double ambiguity = 0;
        g.VISION.tagState = TagFoundState.EMPTY;
        List<PhotonPipelineResult> results = _camera.getAllUnreadResults(); // Get all results from the apriltag pipeline.
        if (!results.isEmpty()) { // If there are no results from the pipeline the results is empty. This happens 2 times. 1. No tag found, 2. Pipeline flushed to often with no new results
            for (PhotonPipelineResult photonPipelineResult : results) {
                if(photonPipelineResult.hasTargets()){
                    List<PhotonTrackedTarget> targets = photonPipelineResult.getTargets();
                    if(!targets.isEmpty()){
                        for (PhotonTrackedTarget target : targets) {
                            ambiguity = target.poseAmbiguity;
                            if(ambiguity >= 0 && ambiguity < g.VISION.ambiguitySetPoint){
                                g.VISION.tagState = TagFoundState.TAG_FOUND;
                                Optional<EstimatedRobotPose> estimatedRobotPose = _poseEstimtor.update(photonPipelineResult);
                                if(estimatedRobotPose.isPresent()){
                                    g.ROBOT.drive.addVisionMeasurement(estimatedRobotPose.get().estimatedPose.toPose2d(), estimatedRobotPose.get().timestampSeconds);
                                    g.VISION.pose2d = estimatedRobotPose.get().estimatedPose.toPose2d();
                                }
                                if(target.getFiducialId() == g.VISION.aprilTagRequestedID){
                                    g.VISION.tagState = TagFoundState.TARGET_ID_FOUND;
                                }
                            }
                        }
                    }
                }
            }
        }
        return g.VISION.tagState;
    }
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
        g.VISION.aprilTagRequestedID = getAprilTagID(g.ROBOT.alignmentState, DriverStation.getAlliance().get());
        TagFoundState leftCamState = calculatePose(m_leftCamera, m_leftPoseEstimator);
        TagFoundState rightCamState = calculatePose(m_rightCamera, m_rightPoseEstimator);
        TagFoundState backCamState = calculatePose(m_backCamera, m_backPoseEstimator);

        if(leftCamState == TagFoundState.TARGET_ID_FOUND || rightCamState == TagFoundState.TARGET_ID_FOUND || backCamState == TagFoundState.TARGET_ID_FOUND){
            g.VISION.isAprilTagFound = true;
        }else if (leftCamState == TagFoundState.EMPTY && rightCamState == TagFoundState.EMPTY && backCamState == TagFoundState.EMPTY){
            g.VISION.isAprilTagFound = false;
            emptyCnt++;
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
        if(g.VISION.aprilTagAlignState != AprilTagAlignState.NONE && g.VISION.isAprilTagFound && g.DRIVETRAIN.isAutoDriveEnabled){
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

    private class VisionPoseEstimatorThread extends Thread{
        public VisionPoseEstimatorThread(){
            super();
        }
        @Override
        public void run(){
            while(true){
                calculatePose();
                try{
                    Thread.sleep(10);
                }catch(InterruptedException e){
                    System.out.println(e.getMessage());
                }
            }
        }
    }
    @Override
    public void updateDashboard() {
        g.VISION.field2d.setRobotPose(g.VISION.pose2d);
        SmartDashboard.putBoolean("Vision/AprilTagIsFound", g.VISION.isAprilTagFound);
        SmartDashboard.putNumber("Vision/Apriltag Requested ID", g.VISION.aprilTagRequestedID);
        SmartDashboard.putNumber("Vision/SetStdDev", emptyCnt);
        //SmartDashboard.putNumber("Vision/AprilTag Requested Pose X", g.VISION.aprilTagRequestedPose.getX());
        //SmartDashboard.putNumber("Vision/AprilTag Requested Pose Y", g.VISION.aprilTagRequestedPose.getY());
        SmartDashboard.putString("Vision/Apriltag AlignState", g.VISION.aprilTagAlignState.toString());
        SmartDashboard.putData("Vision/Vision Field2d", g.VISION.field2d);
        //SmartDashboard.putNumber("Vision/Pose Vision X", g.VISION.pose2d.getX());
        //SmartDashboard.putNumber("Vision/Pose Vision Y", g.VISION.pose2d.getY());

    }
}
