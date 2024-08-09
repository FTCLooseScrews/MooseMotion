package com.loosescrews.follower;

import com.loosescrews.localization.Pose2d;
import com.loosescrews.localization.Vec2d;
import com.loosescrews.path.Path;
import com.loosescrews.util.Angle;
import com.loosescrews.util.NanoClock;
import com.loosescrews.util.PIDFController;

public class Follower {
    public Pose2d lastUpdateVels = null;
    protected Path path;
    protected Pose2d holdingPose = null;
    protected Pose2d lastRobotPose = null;

    private final PIDFController TRANSLATIONAL;
    private final PIDFController DRIVE;
    private final PIDFController HEADING;

    private final double mass;
    private final double scalingf;
    private final double maxVel;
    private final double timeout;
    private final NanoClock clock;
    private double endTime = -1;

    public Follower(PIDFController T, PIDFController D, PIDFController H, double mass, double scalingf, double maxVel, double timeout) {
        this.TRANSLATIONAL = T;
        this.DRIVE = D;
        this.HEADING = H;

        this.mass = mass;
        this.scalingf = scalingf;
        this.maxVel = maxVel;

        this.clock = NanoClock.system();
        this.timeout = timeout;
    }

    public Follower(PIDFController T, PIDFController D, PIDFController H, double mass, double scalingf, double maxVel) {
        this(T, D, H, mass, scalingf, maxVel,2.5);
    }

    public void followPath(Path path) {
        this.path = path;
        this.holdingPose = null;
    }
    public void holdPose(Pose2d pose) {
        this.holdingPose = pose;
        this.path = null;
    }

    public WheelSpeeds update(Pose2d currentRobotPose, Pose2d currentRobotVelocity) {
        if (path == null && holdingPose == null) return null;
        if (path != null && holdingPose != null) holdingPose = null;

        //if we are holding a position
        if (path == null && holdingPose != null) {
            Pose2d translationalVector = getTranslationalVector(currentRobotPose, holdingPose);

            return returnWheelSpeeds(translationalVector.x, translationalVector.y, translationalVector.getHeading(), currentRobotPose.theta);
        }

        Pose2d projectedPose = path.getProjectedPose(currentRobotPose);
        Pose2d translationalVector = getTranslationalVector(currentRobotPose, projectedPose);
        Vec2d centripetalForceVector = getCentripetalForceVector(currentRobotVelocity);

        Vec2d corrective;

        if (centripetalForceVector.plus(translationalVector.vec()).mag > 1) {
            corrective = centripetalForceVector.plus(translationalVector.vec().mul(scale(centripetalForceVector, translationalVector.vec())));
        }
        else {
            corrective = centripetalForceVector.plus(translationalVector.vec());
        }

        Vec2d driveVector = getDriveVector(currentRobotPose, projectedPose);

        //adding timeout for path ending
        if (driveVector == null) {
            if (endTime != -1) {
                double dt = clock.seconds() - endTime;
                if (dt >= timeout) {
                    path = null;
                    endTime = -1;
                }
            }
            else {
                endTime = clock.seconds();
            }

            return returnWheelSpeeds(corrective.x, corrective.y, translationalVector.getHeading(), currentRobotPose.theta);
        }

        if (corrective.mag == 1) {
            return returnWheelSpeeds(corrective.x, corrective.y, translationalVector.getHeading(), currentRobotPose.theta);
        }
        else {
            Vec2d finalVector;
            if (corrective.plus(driveVector).mag > 1) {
                finalVector = corrective.plus(driveVector.mul(scale(corrective, driveVector)));
            }
            else {
                finalVector = corrective.plus(driveVector);
            }
            return returnWheelSpeeds(finalVector.x, finalVector.y, translationalVector.getHeading(), currentRobotPose.theta);
        }
    }

    public WheelSpeeds returnWheelSpeeds(double xVel, double yVel, double thetaVel, double robotHeading) {
        Pose2d chassisSpeeds = Kinematics.fromFieldRelativeSpeeds(xVel, yVel, thetaVel, robotHeading);
        lastUpdateVels = new Pose2d(xVel, yVel, thetaVel);
        return Kinematics.robotSpeedsToWheelSpeeds(chassisSpeeds, 1.0, 1.0);

    }

    private Pose2d getTranslationalVector(Pose2d currentRobotPose, Pose2d projectedPose) {
        Pose2d targetPose = projectedPose;
        Pose2d poseError = targetPose.minus(currentRobotPose);

        double translationalVectorMagnitude = TRANSLATIONAL.calculate(0, poseError.vec().mag);
        Vec2d translationalVector = new Vec2d(Math.min(Math.max(0,translationalVectorMagnitude), 1), poseError.vec().theta);

        double headingError = Angle.getRotationSide(currentRobotPose.theta, targetPose.theta) * Angle.smallestDifference(currentRobotPose.theta, targetPose.theta);
        double headingCorrection = HEADING.calculate(0, headingError);

        return new Pose2d(translationalVector, Math.min(Math.max(-1, headingCorrection), 1));
    }

    private Vec2d getCentripetalForceVector(Pose2d currentRobotVelocity) {
        double curvature = path.projectedPointCurvature();
        if (Double.isNaN(curvature)) return new Vec2d();
        if (currentRobotVelocity == null) return new Vec2d();
        if (curvature == 0) return new Vec2d();

        double centripetalMag = mass * scalingf * Math.pow(currentRobotVelocity.vec().norm()/maxVel, 2) * curvature;

        return new Vec2d(Math.abs(Math.min(Math.max(-1, centripetalMag), 1)),
                (curvature > 0 ? path.projectedPointTangent().theta + Math.PI / 2 :
                        path.projectedPointTangent().theta - Math.PI / 2));
    }


    private Vec2d getDriveVector(Pose2d currentRobotPose, Pose2d projectedPoseOnCurve) {
        Vec2d nextWaypointVec = path.getNextWaypoint(currentRobotPose, lastRobotPose);
        Pose2d current = projectedPoseOnCurve;

        if (nextWaypointVec != null) {
            Vec2d drivePoseDelta = nextWaypointVec.minus(current.vec());
            double driveVectorMagnitude = DRIVE.calculate(0, drivePoseDelta.mag);
            Vec2d driveVector = new Vec2d(Math.min(Math.max(-1,driveVectorMagnitude), 1), drivePoseDelta.theta);

            lastRobotPose = currentRobotPose;
            return driveVector;
        }

        return null;
    }
    public double scale (Vec2d base, Vec2d scaling) {
        double x = base.x;
        double y = base.y;
        double m = scaling.x;
        double n = scaling.y;

        double a = m*m+n*n;
        double b = (x*m+y*n)*2;
        double c = x*x+y*y-1;
        return (-b + Math.sqrt(b*b-4*a*c)) / (2*a);
    }

    public boolean isBusy() {
        return path != null;
    }

    public boolean isHoldingPose() {
        return holdingPose != null;
    }

    public Pose2d getHoldingPose() {
        return holdingPose;
    }

    public Path getCurrentPath() {
        return path;
    }
}
