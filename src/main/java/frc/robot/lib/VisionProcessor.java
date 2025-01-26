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
    EstimatedRobotPose m_estimatedRobotPose;
    public VisionProcessor(){
        camera = new PhotonCamera("leftArducam");
        camera.setPipelineIndex(0);
        camera.setDriverMode(false);
        // TODO: update cameral location on robot. x forward, y left, z up
        Transform3d m_cameraLocation = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
        m_poseEstimator = new PhotonPoseEstimator(m_apriltagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_cameraLocation);

        g.DASHBOARD.updates.add(this);
        
    }
    public void setAprilTagData(){

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        g.VISION.isAprilTagFound = false;
        if (!results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    g.VISION.aprilTagAngle_deg = getYawFromCamera(target.getYaw());
                    double a = target.getBestCameraToTarget().getMeasureX().in(Meter);
                    double b = target.getBestCameraToTarget().getMeasureY().in(Meter);
                    g.VISION.aprilTagDistance_m = Math.sqrt(a * a + b * b);
                    m_estimatedRobotPose = getEstimatedGlobalPose(g.ROBOT.pose2d, result).get();
                    g.ROBOT.drive.resetOdometry(m_estimatedRobotPose.estimatedPose.toPose2d());
                    g.VISION.isAprilTagFound = false;
                    if (target.getFiducialId() == getAprilTagID(g.ROBOT.alignmentState, DriverStation.getAlliance().get())) {
                        g.VISION.isAprilTagFound = true;
                    }
                }
            }
        }
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d _prevEstimatedRobotPose, PhotonPipelineResult _result){
        m_poseEstimator.setReferencePose(_prevEstimatedRobotPose);
        return m_poseEstimator.update(_result);
    }
    private double getYawFromCamera(double _yaw){
        // Determine which camera is being used.
        // 
        double rtn = _yaw;
        switch (g.VISION.aprilTagAlignState) {
            case CENTER:
                // TODO: adjust the yaw to compensate for the use of the right camera
                rtn = _yaw;
                break;
            case RIGHT:
            case LEFT:
            case NONE:
            default:

                break;
        }
        return rtn;
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
    }
}
