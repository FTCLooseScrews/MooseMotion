package com.loosescrews.localization;

import com.loosescrews.follower.Kinematics;
import com.loosescrews.util.Angle;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;
import java.util.List;

public abstract class TwoWheelLocalizer implements Localizer {
    private Pose2d poseEstimate = new Pose2d();
    private Pose2d poseVelocity = null;
    List<Double> lastWheelPositions = new ArrayList<>();
    private double lastHeading = Double.NaN;
    private final DecompositionSolver forwardSolver;

    public TwoWheelLocalizer(List<Pose2d> wheelPoses) {
        assert wheelPoses.size() == 2;

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);
        for (int i = 0; i <= 1; i++) {
            Vec2d orientationVec = wheelPoses.get(i).headingVec();
            Vec2d positionVec = wheelPoses.get(i).vec();
            inverseMatrix.setEntry(i, 0, orientationVec.x);
            inverseMatrix.setEntry(i, 1, orientationVec.y);
            inverseMatrix.setEntry(
                    i,
                    2,
                    positionVec.x * orientationVec.y - positionVec.y * orientationVec.x
            );
        }
        inverseMatrix.setEntry(2, 2, 1.0);

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();
    }

    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public void setPoseEstimate(Pose2d value) {
        lastWheelPositions = new ArrayList<>();
        lastHeading = Double.NaN;
        poseEstimate = value;
    }

    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    private Pose2d calculatePoseDelta(List<Double> wheelDeltas, double headingDelta) {
        wheelDeltas.add(headingDelta);

        double[] l = new double[wheelDeltas.size()];
        int index = 0;
        for (double i : wheelDeltas) {
            l[index] = i;
            index++;
        }

        double[][] arr = {l};

        RealMatrix rawPoseDelta = forwardSolver.solve(
                MatrixUtils.createRealMatrix(
                        arr
                ).transpose()
        );
        return new Pose2d(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        );
    }

    @Override
    public void update() {
        List<Double> wheelPositions = getWheelPositions();
        double heading = getHeading();
        if (!lastWheelPositions.isEmpty()) {
            List<Double> wheelDeltas = new ArrayList<>();
            for (int i = 0; i < wheelPositions.size(); i++) {
                wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
            }
            double headingDelta = Angle.normDelta(heading - lastHeading);
            Pose2d robotPoseDelta = calculatePoseDelta(wheelDeltas, headingDelta);
            poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, robotPoseDelta);
        }

        List<Double> wheelVelocities = getWheelVelocities();
        Double headingVelocity = getHeadingVelocity();
        if (wheelVelocities != null && headingVelocity != null) {
            poseVelocity = calculatePoseDelta(wheelVelocities, headingVelocity);
        }

        lastWheelPositions = wheelPositions;
        lastHeading = heading;
    }

    public abstract List<Double> getWheelPositions();

    public List<Double> getWheelVelocities() {
        return null;
    }

    public abstract double getHeading();

    public Double getHeadingVelocity() {
        return null;
    }
}

