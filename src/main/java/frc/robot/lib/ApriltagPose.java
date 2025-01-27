// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;



public class ApriltagPose {
    double x_left;
    double y_left;
    double x_right;
    double y_right;
    double x_center;
    double y_center;
    double angle;
    /**
     * 
     * @param x_left The field X value for the Pose of the left object or coral posts
     * @param y_left The field Y value for the Pose of the left object or coral posts
     * @param x_right The field X value for the Pose of the right object or coral posts
     * @param y_right The field Y value for the Pose of the right object or coral posts
     * @param x_center The center of the tag from the drawing
     * @param y_center The center of the tag from the drawing
     * @param angle The angle our robot should face for the tag
     */
    public ApriltagPose(double x_left, double y_left, double x_right, double y_right, double x_center, double y_center, double angle) {
        this.x_left = x_left;
        this.y_left = y_left;
        this.x_right = x_right;
        this.y_right = y_right;
        this.x_center = x_center;
        this.y_center = y_center;
        this.angle = angle;
    }
    
}
