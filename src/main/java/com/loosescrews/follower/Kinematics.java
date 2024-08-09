package com.loosescrews.follower;

import com.loosescrews.localization.Pose2d;
import com.loosescrews.localization.Vec2d;
import com.loosescrews.util.Angle;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Kinematics {
    public static WheelSpeeds robotSpeedsToWheelSpeeds(Pose2d vel, double trackWidth, double wheelBase) {
        double n = (trackWidth + wheelBase) / 2.0;
        return new WheelSpeeds(
                vel.x - vel.y - n * vel.theta,
                vel.x + vel.y + n * vel.theta,
                vel.x + vel.y - n * vel.theta,
                vel.x - vel.y + n * vel.theta
        );
    }


    public static Pose2d calculateRobotPoseError(Pose2d targetFieldPose, Pose2d currentFieldPose) {
        Pose2d errorInFieldFrame = calculateFieldPoseError(targetFieldPose, currentFieldPose);
        return new Pose2d(
                errorInFieldFrame.vec().rotated(-currentFieldPose.theta),
                errorInFieldFrame.theta
        );
    }

    //thetaVel is in radians per second and robotHeading is in radians too
    public static Pose2d fromFieldRelativeSpeeds(double xVel, double yVel, double thetaVel, double robotHeading) {
        Vec2d rotated = Vec2d.fromCartesian(xVel, yVel).rotated(-robotHeading);
        return new Pose2d(rotated.x, rotated.y, thetaVel);
    }

    public static Pose2d calculateFieldPoseError(Pose2d target, Pose2d current) {
        return new Pose2d((target.minus(current)).vec(),
                Angle.normDelta(target.theta - current.theta)
        );
    }

    public static Pose2d relativeOdometryUpdate(Pose2d fieldPose, Pose2d robotPoseDelta) {
        double dtheta = robotPoseDelta.theta;

        double sineTerm, cosTerm;

        if (Math.abs(dtheta) <= 1e-6) {
            sineTerm = 1.0 - dtheta * dtheta / 6.0;
            cosTerm = dtheta / 2.0;
        }
        else {
            sineTerm = sin(dtheta) / dtheta;
            cosTerm = (1-cos(dtheta)) / dtheta;
        }

        Vec2d fieldPositionDelta = Vec2d.fromCartesian(
                sineTerm * robotPoseDelta.x - cosTerm * robotPoseDelta.y,
                cosTerm * robotPoseDelta.x + sineTerm * robotPoseDelta.y
        );

        Pose2d fieldPoseDelta = new Pose2d(fieldPositionDelta.rotated(fieldPose.theta), robotPoseDelta.theta);

        return new Pose2d(
                fieldPose.x + fieldPoseDelta.x,
                fieldPose.y + fieldPoseDelta.y,
                Angle.norm(fieldPose.theta + fieldPoseDelta.theta)
        );
    }
}

