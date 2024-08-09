package com.loosescrews.path;

import com.loosescrews.localization.Pose2d;
import com.loosescrews.localization.Vec2d;

public class ProjectedPoint {
    private Vec2d tangent;
    private Vec2d normal;
    private Vec2d point;

    private double t;
    private double c;

    private ProjectedPoint(double t, Vec2d point, Vec2d tangent, Vec2d normal, double c) {
        this.t = t;
        this.point = point;
        this.tangent = tangent;
        this.normal = normal;
        this.c = c;
    }

    public static ProjectedPoint projectFrom(ParametricCurve curve, Pose2d pose) {
        double lower = 0;
        double upper = 1;

        for (int i = 0; i < 30; i++) {
            if (pose.vec().distTo(curve.get(lower + 0.25 * (upper-lower))) >
                    pose.vec().distTo(curve.get(lower + 0.75 * (upper-lower)))) {
                lower += (upper-lower)/2.0;
            } else {
                upper -= (upper-lower)/2.0;
            }
        }

        double t = lower + 0.5 * (upper - lower);

        return new ProjectedPoint(t, curve.get(t), curve.deriv(t), curve.getSecondDerivHeading(t), curve.getCurvature(t));
    }

    public Vec2d getTangent() {
        return tangent;
    }

    public Vec2d getNormal() {
        return normal;
    }

    public double getT() {
        return t;
    }

    public double getC() {
        return c;
    }

    public Vec2d getPoint() {
        return point;
    }
}
