// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/** Add your docs here. */
public class PoseEstimateStatus {
    TagFoundState m_state = TagFoundState.EMPTY;
    double m_ambiguity = -1.0;
    public PoseEstimateStatus(TagFoundState _state, double _ambiguity){
        m_state = _state;
        m_ambiguity = _ambiguity;
    }
    public TagFoundState getState(){
        return m_state;
    }
    public double getAmbiguity(){
        return m_ambiguity;
    }
}
