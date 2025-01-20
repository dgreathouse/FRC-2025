package frc.robot.lib;

import static edu.wpi.first.units.Units.Meter;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**  */
public class VisionProcessor implements IUpdateDashboard{

    PhotonCamera m_leftCamera;
    PhotonCamera m_rightCamera;
    List<PhotonTrackedTarget> m_targets;
    PhotonTrackedTarget m_target;
    public VisionProcessor(){
        m_leftCamera = new PhotonCamera("leftArducam");
        m_leftCamera.setPipelineIndex(0);
        m_leftCamera.setDriverMode(false);
        m_rightCamera = new PhotonCamera("rightArducam");
        m_rightCamera.setPipelineIndex(0);
        m_rightCamera.setDriverMode(false);
        g.DASHBOARD.updates.add(this);
    }
    public void setAprilTagData(){
        PhotonCamera camera = m_rightCamera;
        switch (g.VISION.aprilTagButtonState) {
            case CENTER:
                camera = m_rightCamera;
                break;
            case RIGHT:
                camera = m_leftCamera;
                break;
            case LEFT:
                camera = m_rightCamera;
                break;
            case NONE:
                camera = m_rightCamera;
                break;
            default:
                camera = m_rightCamera;
                break;
        }
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        g.VISION.isAprilTagFound = false;
        if(!results.isEmpty()){
            PhotonPipelineResult result = results.get(results.size() -1);
            if (result.hasTargets()){
                for (PhotonTrackedTarget target: result.getTargets()){
                    g.VISION.aprilTagAngle_deg = getYawFromCamera(target.getYaw());
                    double a = target.getBestCameraToTarget().getMeasureX().in(Meter);
                    double b = target.getBestCameraToTarget().getMeasureY().in(Meter);
                    g.VISION.aprilTagDistance_m = Math.sqrt(a*a + b*b);
                    g.VISION.isAprilTagFound = false;
                    if(target.getFiducialId() == getAprilTagID(g.ROBOT.alignmentState, DriverStation.getAlliance().get())){
                        g.VISION.isAprilTagFound = true;
                    }
                }
            }
        }
    }
    private double getYawFromCamera(double _yaw){
        // Determine which camera is being used.
        // 
        double rtn = _yaw;
        switch (g.VISION.aprilTagButtonState) {
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
        if(g.VISION.aprilTagButtonState != AprilTagButtonState.NONE && g.VISION.isAprilTagFound){
            return true;
        }
        return false;
    }

    // TODO: Finish adding logic for all apriltags
    private static int getAprilTagID(RobotAlignStates _alignState, Alliance _alliance){
        int rtn = 0; // 0 is an invalid ID
        switch(_alignState){
            case BACK:
                // The long if/else approach
                if(_alliance == Alliance.Blue){
                    rtn = 21;
                }else {
                    rtn = 10;
                }
                // Exactly the same but on one line
                if(_alliance == Alliance.Blue){ rtn = 21; }else { rtn = 10; }
                // Shorthand if/else
                rtn = _alliance == Alliance.Blue ? 21 : 10;
                break;
            case BACK_LEFT:
                break;
            case BACK_RIGHT:
                break;
            case FRONT:
                break;
            case FRONT_LEFT:
                break;
            case FRONT_RIGHT:
                break;
            case LEFT:
                break;
            case RIGHT:
                break;
            case STATION_LEFT:
                break;
            case STATION_RIGHT:
                break;
            default:
                break;
            
        }

        return rtn;
        
    }
    @Override
    public void updateDashboard() {
        SmartDashboard.putNumber("Vision/AprilTagYaw_deg", g.VISION.aprilTagAngle_deg);
        SmartDashboard.putNumber("Vision/AprilTagDistance_in", Units.metersToInches(g.VISION.aprilTagDistance_m));
        SmartDashboard.putBoolean("Vision/AprilTagIsFound", g.VISION.isAprilTagFound);
    }
}
