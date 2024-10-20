package com.loosescrews.localization;

import com.loosescrews.follower.Kinematics;
import org.apache.commons.math3.linear.*;

import java.util.ArrayList;
import java.util.List;

public abstract class ThreeWheelLocalizer implements Localizer {
    private Pose2d poseEstimate = new Pose2d();
    private List<Double> lastWheelPositions = new ArrayList<>();
    private Pose2d poseVelocity = null;

    private final DecompositionSolver forwardSolver;

    public ThreeWheelLocalizer(List<Pose2d> wheelPoses) {
        assert wheelPoses.size() == 3;

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);
        for (int i = 0; i <= 2; i++) {
            Vec2d orientationVector = wheelPoses.get(i).headingVec();
            Vec2d positionVector = wheelPoses.get(i).vec();
            inverseMatrix.setEntry(i, 0, orientationVector.x);
            inverseMatrix.setEntry(i, 1, orientationVector.y);
            inverseMatrix.setEntry(
                    i,
                    2,
                    positionVector.x * orientationVector.y - positionVector.y * orientationVector.x
            );
        }

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();
    }

    private Pose2d calculatePoseDelta(List<Double> wheelDeltas) {
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

    public void update() {
        List<Double> wheelPositions = getWheelPositions();

        if (!lastWheelPositions.isEmpty()) {
            List<Double> wheelDeltas = new ArrayList<>();
            for (int i = 0; i < wheelPositions.size(); i++) {
                wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
            }
            Pose2d robotPoseDelta = calculatePoseDelta(wheelDeltas);
            poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, robotPoseDelta);
        }

        List<Double> wheelVelocities = getWheelVelocities();
        if (wheelVelocities != null) {
            poseVelocity = calculatePoseDelta(wheelVelocities);
        }

        lastWheelPositions = wheelPositions;
    }

    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public void setPoseEstimate(Pose2d estimate) {
        lastWheelPositions = new ArrayList<>();
        poseEstimate = estimate;
    }

    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    public abstract List<Double> getWheelPositions();
    public List<Double> getWheelVelocities() {
        return null;
    }
}
