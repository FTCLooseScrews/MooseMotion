package com.loosescrews.path;

import com.loosescrews.localization.Pose2d;
import com.loosescrews.localization.Vec2d;

import java.util.ArrayList;
import java.util.Collections;

public class Bezier extends ParametricCurve {
    private final ArrayList<Vec2d> controlPoints = new ArrayList<>();
    private final BezierCurve x;
    private final BezierCurve y;

    public Bezier(Vec2d... points) {
        Collections.addAll(controlPoints, points);

        ArrayList<Double> xp = new ArrayList<>();
        ArrayList<Double> yp = new ArrayList<>();
        for (Vec2d p : controlPoints) {
            xp.add(p.x);
            yp.add(p.y);
        }

        x = new BezierCurve(xp);
        y = new BezierCurve(yp);

        for (int i = 1; i <= 1000; i++) {
            waypoints.add(new Waypoint(Vec2d.fromCartesian(x.get(i/1000.0), y.get(i/1000.0)), i/1000.0));
        }
    }
    @Override
    public Vec2d end() {
        return controlPoints.get(controlPoints.size()-1);
    }

    @Override
    public Waypoint getNextWaypoint(Pose2d pose, Pose2d last) {
        ProjectedPoint point = ProjectedPoint.projectFrom(this, pose);

        if (point.getT() == 0) return waypoints.get(0);

        for (int i = 0; i < waypoints.size(); i++) {
            if (waypoints.get(i).getT() >= point.getT()) {
                if (i >= 995) {
                    return null;
                }
                if (waypoints.get(i).getWaypointVec().distTo(pose.vec()) <=
                        waypoints.get(i-1).getWaypointVec().distTo(pose.vec())) {
                    return waypoints.get(i == waypoints.size()-1 ? i : i+1);
                }
                return waypoints.get(i);
            }
        }

        return null;
    }
    public Vec2d get(double t) {
        return Vec2d.fromCartesian(x.get(t), y.get(t));
    }

    public Vec2d deriv(double t) {
        return Vec2d.fromCartesian(x.deriv(t), y.deriv(t));
    }

    public Vec2d secondDeriv(double t) {
        return Vec2d.fromCartesian(x.secondDeriv(t), y.secondDeriv(t));
    }

    public Vec2d getSecondDerivHeading(double t) {
        double derivAngle = deriv(t).getAngle();
        double deltaAngle = deriv(t+0.001).getAngle();

        return new Vec2d(1, deltaAngle-derivAngle);
    }

    public double getCurvature(double t) {
        t = Math.min(Math.max(0, t), 1);
        Vec2d deriv = deriv(t);
        Vec2d secondDeriv = secondDeriv(t);

        if (deriv.norm() == 0) return 0;
        return (deriv.x*secondDeriv.y-deriv.y-secondDeriv.x)/Math.pow(deriv.norm(), 3);
    }

    public double length() {
        return lengthFromWaypoint(0);
    }

    public double lengthFromWaypoint(int wpIndex) {
        if (wpIndex == waypoints.size() || wpIndex == waypoints.size()-1) return 0;

        double sum = 0;
        for (int i = wpIndex+1; i < waypoints.size(); i++) {
            sum += waypoints.get(i).getWaypointVec().distTo(waypoints.get(i-1).getWaypointVec());
        }
        return sum;
    }

    private static class BezierCurve {
        public ArrayList<Double> controlPoints;

        public BezierCurve(ArrayList<Double> points) {
            this.controlPoints = points;
        }

        public double get(double t) {
            double ret = 0;
            double n = controlPoints.size()-1;
            for (int i = 0; i <= n; i++) {
                ret += B(n, i, t) * controlPoints.get(i);
            }
            return ret;
        }

        public double deriv(double t) {
            double ret = 0;
            double n = controlPoints.size()-1;
            for (int i = 0; i <= n-1; i++) {
                ret += B(n-1, i, t) * n * (controlPoints.get(i+1) - controlPoints.get(i));
            }
            return ret;
        }

        public double secondDeriv(double t) {
            double ret = 0;
            double n = controlPoints.size()-1;
            for (int i = 0; i <= n-2; i++) {
                ret += B(n-2, i, t) * n * (n-1) * (controlPoints.get(i+2) - 2 * controlPoints.get(i+1) + controlPoints.get(i));
            }
            return ret;
        }

        public double B(double n, double i, double t) {
            return nCr(n, i) * Math.pow(t, i) * Math.pow(1-t, n-i);
        }

        private double nCr(double n, double r) {
            double ret = 1;
            for (int i = 1; i <= r; i++) {
                ret = ret * (n - r + i) / i;
            }
            return ((int)ret);
        }
    }
}
