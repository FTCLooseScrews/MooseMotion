package com.loosescrews.path;

import com.loosescrews.localization.Pose2d;
import com.loosescrews.localization.Vec2d;
import com.loosescrews.util.Angle;

public class Path {
    private ParametricCurve curve;
    private ProjectedPoint projectedPoint;
    private double speedConstraint;

    private double startHeading;
    private double endHeading;
    private boolean isTangentHeadingInterpolation = true;
    private boolean holdEnd = false;

    public Path(ParametricCurve curve, double speedConstraint) {
        this.curve = curve;
        this.speedConstraint = speedConstraint;
    }

    public Path(ParametricCurve curve, double speedConstraint, boolean holdEnd) {
        this(curve, speedConstraint);
        this.holdEnd = holdEnd;
    }

    public Path(ParametricCurve curve, boolean holdEnd) {
        this(curve);
        this.holdEnd = holdEnd;
    }


    public Path(ParametricCurve curve) {
        this(curve, 0.9);
    }

    public ParametricCurve.Waypoint getNextWaypoint(Pose2d currentPose, Pose2d lastPose) {
        return curve.getNextWaypoint(currentPose, lastPose);
    }

    public Pose2d getProjectedPose(Pose2d pose) {
        projectedPoint = ProjectedPoint.projectFrom(curve, pose);
        return new Pose2d(projectedPoint.getPoint(), projectedPointHeading());
    }

    public Pose2d start() {
        if (isTangentHeadingInterpolation) {
            return new Pose2d(curve.get(0), curve.deriv(0).getAngle());
        }
        return new Pose2d(curve.get(0), startHeading);
    }

    public Pose2d end() {
        if (isTangentHeadingInterpolation) {
            return new Pose2d(curve.get(1), curve.deriv(1).getAngle());
        }
        return new Pose2d(curve.get(1), endHeading);
    }

    public Path setLinearHeadingInterpolation(double startHeading, double endHeading) {
        isTangentHeadingInterpolation = false;
        this.startHeading = startHeading;
        this.endHeading = endHeading;
        return this;
    }

    public Path setConstantHeadingInterpolation(double setHeading) {
        isTangentHeadingInterpolation = false;
        this.startHeading = setHeading;
        this.endHeading = setHeading;
        return this;
    }

    public Path setTangentHeadingInterpolation() {
        isTangentHeadingInterpolation = true;
        return this;
    }

    public Path setSpeedConstraint(double speed) {
        speedConstraint = speed;
        return this;
    }

    public double projectedPointHeading() {
        if (isTangentHeadingInterpolation) {
            return projectedPoint.getTangent().getAngle();
        }
        return Angle.norm(startHeading + Angle.getRotationSide(startHeading, endHeading)*Angle.smallestDifference(endHeading, startHeading)*projectedPoint.getT());
    }

    public Vec2d projectedPointNormal() {
        return projectedPoint.getNormal();
    }

    public double projectedPointCurvature() {
        return projectedPoint.getC();
    }

    public Vec2d projectedPointTangent() {
        return projectedPoint.getTangent();
    }

    public ParametricCurve getCurve() {
        return curve;
    }

    public double getSpeedConstraint() {
        return speedConstraint;
    }

    public boolean isHoldingEnd() {
        return holdEnd;
    }

    public void setHoldingEnd(boolean holdingEnd) {
        this.holdEnd = holdingEnd;
    }
}
