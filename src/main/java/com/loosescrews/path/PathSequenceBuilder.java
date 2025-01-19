package com.loosescrews.path;

import com.loosescrews.localization.Pose2d;
import com.loosescrews.localization.Vec2d;
import com.loosescrews.path.exceptions.EmptyPathSequenceException;

import java.util.ArrayList;

public class PathSequenceBuilder {
    private ArrayList<Path> sequence;
    private boolean holdLast = false;
    private Pose2d lastPose;

    public PathSequenceBuilder(Pose2d startPose) {
        sequence = new ArrayList<>();
        lastPose = startPose;
    }

    public PathSequenceBuilder lineTo(Vec2d endPosition) {
        return lineTo(endPosition, 0.9);
    }

    public PathSequenceBuilder lineToConstantHeading(Vec2d endPosition, double endHeading) {
        return lineToConstantHeading(endPosition, endHeading, 0.9);
    }

    public PathSequenceBuilder lineToLinearHeading(Vec2d endPosition, double endHeading) {
        return lineToLinearHeading(endPosition, endHeading, 0.9);
    }

    public PathSequenceBuilder cubicBezierSplineTo(Vec2d c2, Vec2d c3, Vec2d end) {
        return cubicBezierSplineTo(c2, c3, end, 0.9);
    }

    public PathSequenceBuilder cubicBezierSplineToConstantHeading(Vec2d c2, Vec2d c3, Vec2d end, double endHeading) {
        return cubicBezierSplineToConstantHeading(c2, c3, end, endHeading, 0.9);
    }

    public PathSequenceBuilder cubicBezierSplineToLinearHeading(Vec2d c2, Vec2d c3, Vec2d end, double endHeading) {
        return cubicBezierSplineToLinearHeading(c2, c3, end, endHeading, 0.9);
    }

    public PathSequenceBuilder bezierSplineTo(Vec2d... points) {
        return bezierSplineTo(0.9, points);
    }

    public PathSequenceBuilder bezierSplineToConstantHeading(double endHeading, Vec2d... points) {
        return bezierSplineToConstantHeading(endHeading, 0.9, points);
    }

    public PathSequenceBuilder bezierSplineToLinearHeading(double endHeading, Vec2d... points) {
        return bezierSplineToLinearHeading(endHeading, 0.9, points);
    }

    public PathSequenceBuilder lineTo(Vec2d endPosition, double speed) {
        Line l = new Line(lastPose.x, lastPose.y, endPosition.x, endPosition.y);
        sequence.add(new Path(l).setTangentHeadingInterpolation().setSpeedConstraint(speed));

        lastPose = new Pose2d(endPosition, l.deriv(1).getAngle());

        return this;
    }

    public PathSequenceBuilder lineToConstantHeading(Vec2d endPosition, double endHeading, double speed) {
        sequence.add(new Path(new Line(lastPose.x, lastPose.y, endPosition.x, endPosition.y))
                .setConstantHeadingInterpolation(endHeading)
                .setSpeedConstraint(speed)
        );

        lastPose = new Pose2d(endPosition, endHeading);

        return this;
    }

    public PathSequenceBuilder lineToLinearHeading(Vec2d endPosition, double endHeading, double speed) {
        sequence.add(new Path(new Line(lastPose.x, lastPose.y, endPosition.x, endPosition.y))
                .setLinearHeadingInterpolation(lastPose.theta, endHeading)
                .setSpeedConstraint(speed)
        );

        lastPose = new Pose2d(endPosition, endHeading);

        return this;
    }

    public PathSequenceBuilder cubicBezierSplineTo(Vec2d c2, Vec2d c3, Vec2d end, double speed) {
        CubicBezier spline = new CubicBezier(lastPose.vec(), c2, c3, end);
        sequence.add(new Path(spline).setTangentHeadingInterpolation().setSpeedConstraint(speed));

        lastPose = new Pose2d(end, spline.deriv(1).getAngle());

        return this;
    }

    public PathSequenceBuilder cubicBezierSplineToConstantHeading(Vec2d c2, Vec2d c3, Vec2d end, double endHeading, double speed) {
        CubicBezier spline = new CubicBezier(lastPose.vec(), c2, c3, end);
        sequence.add(new Path(spline).setConstantHeadingInterpolation(endHeading).setSpeedConstraint(speed));

        lastPose = new Pose2d(end, endHeading);

        return this;
    }

    public PathSequenceBuilder cubicBezierSplineToLinearHeading(Vec2d c2, Vec2d c3, Vec2d end, double endHeading, double speed) {
        CubicBezier spline = new CubicBezier(lastPose.vec(), c2, c3, end);
        sequence.add(new Path(spline).setLinearHeadingInterpolation(lastPose.theta, endHeading).setSpeedConstraint(speed));

        lastPose = new Pose2d(end, endHeading);

        return this;
    }

    public PathSequenceBuilder bezierSplineTo(double speed, Vec2d... points) {
        if (points.length == 0) throw new EmptyPathSequenceException();
        Bezier spline = new Bezier(addStartingPose(points));
        sequence.add(new Path(spline).setTangentHeadingInterpolation().setSpeedConstraint(speed));

        lastPose = new Pose2d(points[points.length-1], spline.deriv(1).getAngle());

        return this;
    }

    public PathSequenceBuilder bezierSplineToConstantHeading(double endHeading, double speed, Vec2d... points) {
        if (points.length == 0) throw new EmptyPathSequenceException();
        Bezier spline = new Bezier(addStartingPose(points));
        sequence.add(new Path(spline).setConstantHeadingInterpolation(endHeading).setSpeedConstraint(speed));

        lastPose = new Pose2d(points[points.length-1], endHeading);

        return this;
    }

    public PathSequenceBuilder bezierSplineToLinearHeading(double endHeading, double speed, Vec2d... points) {
        if (points.length == 0) throw new EmptyPathSequenceException();
        Bezier spline = new Bezier(addStartingPose(points));
        sequence.add(new Path(spline).setLinearHeadingInterpolation(lastPose.theta, endHeading).setSpeedConstraint(speed));

        lastPose = new Pose2d(points[points.length-1], endHeading);

        return this;
    }

    public PathSequenceBuilder holdLast(boolean holding) {
        this.holdLast = holding;
        return this;
    }


    // Do not use if this is not the first method in your path sequence.
    public PathSequenceBuilder addPath(Path path) {
        sequence.add(path);

        lastPose = path.end();

        return this;
    }

    private Vec2d[] addStartingPose(Vec2d[] controlPoints) {
        Vec2d[] ret = new Vec2d[controlPoints.length+1];
        ret[0] = lastPose.vec();
        System.arraycopy(controlPoints, 0, ret, 1, controlPoints.length);
        return ret;
    }

    public PathSequence build() {
        Path end = sequence.get(sequence.size()-1);
        end.setHoldingEnd(holdLast);
        sequence.set(sequence.size()-1, end);

        return new PathSequence(sequence);
    }
}
