package frc.robot.subsystems;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;

public class Coral extends SubsystemBase implements IUpdateDashboard{
  ServoHub m_servoHub;
  ServoChannel m_servo;
  /** Creates a new CoralIntake. */
  public Coral() {
    m_servoHub = new ServoHub(3);
    m_servo = m_servoHub.getServoChannel(ChannelId.kChannelId0);
    m_servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 5000);
    m_servo.setPowered(true);
    m_servo.setEnabled(true);
    g.DASHBOARD.updates.add(this);
    SmartDashboard.putNumber("Servo", 2500);
  }

  @Override
  public void periodic() {

  }
  public void setServo(int _us){
    m_servo.setPulseWidth(_us);
  }
  @Override
  public void updateDashboard() {
    SmartDashboard.putString("Coral/Claw State", g.CORAL.clawState.toString());
  }
}
