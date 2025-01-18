package frc.robot.lib;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**  */
public class VisionProcessor implements IUpdateDashboard{

    PhotonCamera m_camera;
    List<PhotonTrackedTarget> m_targets;
    PhotonTrackedTarget m_target;
    public VisionProcessor(){
        m_camera = new PhotonCamera("arducam");
        m_camera.setPipelineIndex(0);
        m_camera.setDriverMode(false);
        g.DASHBOARD.updates.add(this);
    }
    public void setAprilTagData(){
        var results = m_camera.getAllUnreadResults();
        if(!results.isEmpty()){
            var result = results.get(results.size() -1);
            if (result.hasTargets()){
                for (var target: result.getTargets()){
                    if(target.getFiducialId() == g.VISION.aprilTagIDRequested){
                        g.VISION.aprilTagIsFound = true;
                        g.VISION.aprilTagAngle_deg = target.getYaw();
                        g.VISION.aprilTagDistance_m = PhotonUtils.calculateDistanceToTargetMeters(0.3048, 0.3048, 0, 0);

                    }else {
                        g.VISION.aprilTagIsFound = false;
                    }
                }
            }
            
        }
    }
    // TODO: Finish adding logic for all apriltags
    public static int getAprilTagID(RobotAlignStates _alignState, Alliance _alliance){
        int rtn = 0; 
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
        SmartDashboard.putBoolean("Vision/AprilTagIsFound", g.VISION.aprilTagIsFound);
    }
}
