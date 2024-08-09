package com.loosescrews.localization;

public interface Localizer {
    Pose2d getPoseEstimate();
    void setPoseEstimate(Pose2d estimate);
    Pose2d getPoseVelocity();
    void update();
}
