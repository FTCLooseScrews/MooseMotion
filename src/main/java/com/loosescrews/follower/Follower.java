package com.loosescrews.follower;

import com.loosescrews.localization.Pose2d;
import com.loosescrews.localization.Vec2d;
import com.loosescrews.path.ParametricCurve;
import com.loosescrews.path.Path;
import com.loosescrews.path.PathSequence;
import com.loosescrews.util.Angle;
import com.loosescrews.util.NanoClock;
import com.loosescrews.util.PIDFController;

public class Follower {
    //Debugging
    public Pose2d lastUpdateVels = new Pose2d();
    public Vec2d lastDriveVec = new Vec2d();
    public Pose2d lastTranslationalVec = new Pose2d();
    public Vec2d lastCentripetalVec = new Vec2d();
    public ParametricCurve.Waypoint nextWaypoint = new ParametricCurve.Waypoint(new Vec2d(), 0);
    public double lastLoopTime = -1;

    protected PathSequence pathSequence;
    private int currentIndex = 0;
    protected Path path;
    protected Pose2d holdingPose = null;
    protected Pose2d lastRobotPose = null;
    private boolean isBusy = false;

    private final PIDFController TRANSLATIONAL;
    private final PIDFController DRIVE;
    private final PIDFController HEADING;

    //For centripetal force
    private final double mass;
    private final double scalingf;
    private final double maxVel;

    //For path ending
    private final double forwardDeceleration;
    private final double lateralDeceleration;

    private final double timeout;
    private final NanoClock clock;
    private double endTime = -1;

    public Follower(PIDFController T, PIDFController D, PIDFController H, double mass, double scalingf, double maxVel, double forwardDeceleration, double lateralDeceleration, double timeout) {
        this.TRANSLATIONAL = T;
        this.DRIVE = D;
        this.HEADING = H;

        this.mass = mass;
        this.scalingf = scalingf;
        this.maxVel = maxVel;

        this.forwardDeceleration = forwardDeceleration;
        this.lateralDeceleration = lateralDeceleration;

        this.clock = NanoClock.system();
        this.timeout = timeout;
    }

    public Follower(PIDFController T, PIDFController D, PIDFController H, double mass, double scalingf, double forwardDeceleration, double lateralDeceleration, double maxVel) {
        this(T, D, H, mass, scalingf, maxVel, forwardDeceleration, lateralDeceleration, 1.2);
    }

    public void followPath(Path path) {
        this.path = path;
        this.pathSequence = null;
        this.holdingPose = null;
        this.isBusy = true;
    }
    public void holdPose(Pose2d pose) {
        this.holdingPose = pose;
        this.pathSequence = null;
        this.path = null;
        this.isBusy = true;
    }
    public void followPathSequence(PathSequence sequence) {
        this.pathSequence = sequence;
        this.path = null;
        this.holdingPose = null;
        this.isBusy = true;
        currentIndex = 0;
    }

    public WheelSpeeds update(Pose2d currentRobotPose, Pose2d currentRobotVelocity) {
        if (path == null && holdingPose == null && pathSequence == null) return null;
        if ((path != null || pathSequence != null) && holdingPose != null) holdingPose = null;
        if (path != null && pathSequence != null) path = null;

        //if we are holding a position
        if (path == null && pathSequence == null && holdingPose != null) {
            Pose2d translationalVector = getTranslationalVector(currentRobotPose, holdingPose);
            return returnWheelSpeeds(translationalVector.x, translationalVector.y, translationalVector.getHeading(), currentRobotPose.theta);
        }

        //if we are running a path/at the final segment of pathsequence
        if (path != null) {
            return getVector(path, currentRobotPose, currentRobotVelocity, true);
        }
        if (pathSequence != null) {
            if (currentIndex == pathSequence.size()-1) {
                return getVector(pathSequence.get(currentIndex), currentRobotPose, currentRobotVelocity, true);
            }
            return getVector(pathSequence.get(currentIndex), currentRobotPose, currentRobotVelocity, false);
        }
        return null;
    }

    public WheelSpeeds getVector(Path activePath, Pose2d currentRobotPose, Pose2d currentRobotVelocity, boolean finalPath) {
        Pose2d projectedPose = activePath.getProjectedPose(currentRobotPose);
        Pose2d translationalVector = getTranslationalVector(currentRobotPose, projectedPose);
        Vec2d centripetalForceVector = getCentripetalForceVector(activePath, currentRobotVelocity);

        Vec2d corrective;

        if (centripetalForceVector.plus(translationalVector.vec()).mag > 1) {
            corrective = centripetalForceVector.plus(translationalVector.vec().mul(scale(centripetalForceVector, translationalVector.vec())));
        }
        else {
            corrective = centripetalForceVector.plus(translationalVector.vec());
        }

        Vec2d driveVector = getDriveVector(activePath, currentRobotPose, projectedPose, currentRobotVelocity, finalPath);

        //adding timeout for path ending
        if (finalPath && driveVector == null) {
            if (endTime != -1) {
                double dt = clock.seconds() - endTime;
                if (dt >= timeout) {
                    //RESET EVERY VARIABLE
                    path = null;
                    pathSequence = null;
                    currentIndex = 0;
                    isBusy = false;
                    endTime = -1;
                    lastLoopTime = -1;
                    return null;
                }
            }
            else {
                endTime = clock.seconds();
            }
            lastDriveVec = new Vec2d();
            return returnWheelSpeeds(corrective.x, corrective.y, translationalVector.getHeading(), currentRobotPose.theta);
        }
        if (driveVector == null) {
            currentIndex++;
            return returnWheelSpeeds(corrective.x, corrective.y, translationalVector.getHeading(), currentRobotPose.theta);
        }

        if (corrective.mag == 1) {
            lastTranslationalVec = translationalVector;
            lastCentripetalVec = centripetalForceVector;
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

            lastDriveVec = driveVector;
            lastTranslationalVec = translationalVector;
            lastCentripetalVec = centripetalForceVector;
            return returnWheelSpeeds(finalVector.x, finalVector.y, translationalVector.getHeading(), currentRobotPose.theta);
        }
    }

    public WheelSpeeds returnWheelSpeeds(double xVel, double yVel, double thetaVel, double robotHeading) {
        Pose2d chassisSpeeds = Kinematics.fromFieldRelativeSpeeds(xVel, yVel, thetaVel, robotHeading);
        lastUpdateVels = new Pose2d(xVel, yVel, thetaVel);
        lastLoopTime = clock.seconds();
        return Kinematics.robotSpeedsToWheelSpeeds(chassisSpeeds, 1.0, 1.0);

    }

    private Pose2d getTranslationalVector(Pose2d currentRobotPose, Pose2d projectedPose) {

        //the projected pose is the target pose
        Pose2d poseError = projectedPose.minus(currentRobotPose);

        double translationalVectorMagnitude = TRANSLATIONAL.calculate(0, poseError.vec().mag);
        Vec2d translationalVector = new Vec2d(clamp(0, 1, translationalVectorMagnitude), poseError.vec().theta);

        double headingError = Angle.getRotationSide(currentRobotPose.theta, projectedPose.theta) * Angle.smallestDifference(currentRobotPose.theta, projectedPose.theta);
        double headingCorrection = HEADING.calculate(0, headingError);

        return new Pose2d(translationalVector, clamp(-1, 1, headingCorrection));
    }

    private Vec2d getCentripetalForceVector(Path activePath, Pose2d currentRobotVelocity) {
        double curvature = activePath.projectedPointCurvature();
        if (Double.isNaN(curvature)) return new Vec2d();
        if (currentRobotVelocity == null) return new Vec2d();
        if (curvature == 0) return new Vec2d();
    
//        double centripetalMag = mass * scalingf * Math.pow(currentRobotVelocity.vec().norm()/maxVel, 2) * curvature;
        double deltaT = lastLoopTime == -1 ? 0 : clock.seconds() - lastLoopTime;
        double centripetalMovement = deltaT * Math.pow(currentRobotVelocity.vec().norm(), 2)*curvature/2.0;
        //The scaling factor is just to account for weighting this vector with the translational one. Nothing to do with units or normalizing
        // DIRECTION NEEDED
        return new Vec2d(clamp(0, 1, centripetalMovement * scalingf),
                (curvature > 0 ? activePath.projectedPointTangent().theta + Math.PI / 2 :
                        activePath.projectedPointTangent().theta - Math.PI / 2));
    }


    private Vec2d getDriveVector(Path activePath, Pose2d currentRobotPose, Pose2d projectedPoseOnCurve, Pose2d currentRobotVel, boolean finalPath) {
        nextWaypoint = activePath.getNextWaypoint(currentRobotPose, lastRobotPose);
        Vec2d nextWaypointVec = nextWaypoint.getWaypointVec();

        if (nextWaypointVec != null) {
            //projected pose is where the robot is currently supposed to be
            Vec2d drivePoseDelta = nextWaypointVec.minus(projectedPoseOnCurve.vec());

            double driveVectorMagnitude = finalPath ? DRIVE.calculate(0, drivePoseDelta.mag) : 0.9;
            Vec2d driveVector = new Vec2d(clamp(-1, 1, driveVectorMagnitude), drivePoseDelta.theta);

            if (nextWaypoint.getT() > 0.85 && currentRobotVel != null) {
                double xDist = sign(activePath.end().vec().minus(projectedPoseOnCurve.vec()).x) * Math.sqrt(-currentRobotVel.x*currentRobotVel.x / (2 * forwardDeceleration));
                double yDist = sign(activePath.end().vec().minus(projectedPoseOnCurve.vec()).y) * Math.sqrt(-currentRobotVel.y * currentRobotVel.y / (2 * lateralDeceleration));

                Vec2d zeroPowerVec = Vec2d.fromCartesian(xDist,yDist).plus(currentRobotPose.vec());
                if (zeroPowerVec.distTo(projectedPoseOnCurve.vec()) > activePath.end().vec().distTo(projectedPoseOnCurve.vec())) {
                    Vec2d zeroPowerGoalCorrection = activePath.end().vec().minus(zeroPowerVec);
                    double correctionMag = DRIVE.calculate(0, zeroPowerGoalCorrection.mag/1000);
                    Vec2d correctionVec = new Vec2d(clamp(-1, 1, correctionMag), zeroPowerGoalCorrection.theta);

                    //this shouldn't happen but just in case
                    driveVector = driveVector.plus(correctionVec).mag > 1 ?
                            driveVector.plus(correctionVec.mul(scale(driveVector, correctionVec))) : driveVector.plus(correctionVec);
                }
            }

            lastRobotPose = currentRobotPose;
            return driveVector;
        }

        return null;
    }

    public double sign(double num) {
        return num > 0 ? 1 : num < 0 ? -1 : 0;
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
        return isBusy;
    }

    public boolean isHoldingPose() {
        return holdingPose != null;
    }

    public Pose2d getHoldingPose() {
        return holdingPose;
    }

    public Path getActivePath() {
        if (pathSequence != null) {
            return pathSequence.get(currentIndex);
        }
        return path;
    }

    public PathSequence getCurrentPathSequence() {
        return pathSequence;
    }
    public static double clamp(double min, double max, double val) {
        return Math.min(Math.max(min, val), max);
    }
}
