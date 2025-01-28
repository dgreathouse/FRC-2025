package frc.robot.lib;

import static edu.wpi.first.units.Units.Meter;
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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**  */
public class VisionProcessor implements IUpdateDashboard{

    PhotonCamera camera;
    List<PhotonTrackedTarget> m_targets;
    PhotonTrackedTarget m_target;
    AprilTagFieldLayout m_apriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    PhotonPoseEstimator m_poseEstimator;
    Optional<EstimatedRobotPose> m_estimatedRobotPose;
    public VisionProcessor(){
        camera = new PhotonCamera("leftArducam");
        camera.setPipelineIndex(0);
        camera.setDriverMode(false);
        // TODO: update cameral location on robot. x forward, y left, z up
        Transform3d m_cameraLocation = new Transform3d(new Translation3d(0.33,0,0.33), new Rotation3d(0,0,0));
        m_poseEstimator = new PhotonPoseEstimator(m_apriltagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_cameraLocation);

        g.DASHBOARD.updates.add(this);
        createApriltagLocations();
        
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
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 1 red left station. Only care about center at this point
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 2 red right station. Only care about center at this point
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 3 red processor. Only care about center at this point
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 4
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 5
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 6
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 7
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 8
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 9
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 10
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 11
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 12
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 13
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 14
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 15
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 16
        x = m_apriltagFieldLayout.getTagPose(17).get().getX();
        y = m_apriltagFieldLayout.getTagPose(17).get().getY();
        cx = x-g.ROBOT.centerDistanceToFrontBumper_mm/2;
        cy = y-cx*1.732;
        g.AprilTagLocations.pose.add(new ApriltagPose(cx-(g.FIELD.TAG_TO_POST_mm*0.866), cy+g.FIELD.TAG_TO_POST_mm/2, 0, 0, cx, cy, 0));  // ID 17
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 18
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 19
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 20
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 21
        g.AprilTagLocations.pose.add(new ApriltagPose(0, 0, 0, 0, 0, 0, 0));  // ID 22


    }
    public void setAprilTagData(){

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
       
        if (!results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    g.VISION.aprilTagAngle_deg = target.getYaw();
                    double a = target.getBestCameraToTarget().getMeasureX().in(Meter);
                    double b = target.getBestCameraToTarget().getMeasureY().in(Meter);
                    g.VISION.aprilTagDistance_m = Math.sqrt(a * a + b * b);
                    m_estimatedRobotPose = getEstimatedGlobalPose(g.ROBOT.pose2d,result);
                    if(m_estimatedRobotPose.isPresent()){
                        g.ROBOT.drive.resetOdometry(m_estimatedRobotPose.get().estimatedPose.toPose2d());
                    }
                    int id = getAprilTagID(g.ROBOT.alignmentState, DriverStation.getAlliance().get());
                    if (target.getFiducialId() == id) {
                        g.VISION.isAprilTagFound = true;
                        Optional<Pose3d> requestedPose = m_apriltagFieldLayout.getTagPose(id);
                        if(requestedPose.isPresent()){
                            g.VISION.aprilTagRequestedPose = requestedPose;
                        }

                    }else {
                        g.VISION.isAprilTagFound = false;
                    }
                }
            }else {
                g.VISION.isAprilTagFound = false;
            }
        }
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d _prevEstimatedRobotPose, PhotonPipelineResult _result){
        m_poseEstimator.setReferencePose(_prevEstimatedRobotPose);
        return m_poseEstimator.update(_result);
    }

    public boolean getIsAutoAprilTagActive(){
        if(g.VISION.aprilTagAlignState != AprilTagAlignState.NONE && g.VISION.isAprilTagFound){
            return true;
        }
        return false;
    }
    
    public static int getAprilTagID(){
        return getAprilTagID(g.ROBOT.alignmentState, DriverStation.getAlliance().get());
    }

    public static int getAprilTagID(RobotAlignStates _alignState, Alliance _alliance) {
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
    public void calculate(){
        setAprilTagData();
    }

    @Override
    public void updateDashboard() {
        SmartDashboard.putNumber("Vision/AprilTagYaw_deg", g.VISION.aprilTagAngle_deg);
        SmartDashboard.putNumber("Vision/AprilTagDistance_m", g.VISION.aprilTagDistance_m);
        SmartDashboard.putBoolean("Vision/AprilTagIsFound", g.VISION.isAprilTagFound);
        if(g.VISION.aprilTagRequestedPose.isPresent()){
            SmartDashboard.putNumber("Vision/AprilTag Requested Pose X", g.VISION.aprilTagRequestedPose.get().getX());
            SmartDashboard.putNumber("Vision/AprilTag Requested Pose Y", g.VISION.aprilTagRequestedPose.get().getY());
        }
    }
}
