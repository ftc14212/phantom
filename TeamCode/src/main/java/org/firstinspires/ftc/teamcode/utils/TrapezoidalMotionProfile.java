package org.firstinspires.ftc.teamcode.utils;

public class TrapezoidalMotionProfile {

    private double maxVel;
    private double maxAccel;

    private double profiledPosition = 0;
    private double currentVelocity = 0;

    private long lastTime;

    public TrapezoidalMotionProfile(double maxVel, double maxAccel) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;

        lastTime = System.nanoTime();
    }

    public void setConstraints(double maxVel, double maxAccel) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
    }

    public void reset(double position) {
        profiledPosition = position;
        currentVelocity = 0;
        lastTime = System.nanoTime();
    }

    public double update(double targetPosition) {

        long now = System.nanoTime();

        double dt = (now - lastTime) / 1e9;

        lastTime = now;

        if (dt <= 0) dt = 0.001;

        double error = targetPosition - profiledPosition;

        // desired velocity
        double desiredVelocity =
                Math.signum(error) * maxVel;

        // stopping distance
        double stoppingDistance =
                (currentVelocity * currentVelocity)
                        / (2.0 * maxAccel);

        // decelerate near target
        if (Math.abs(error) <= stoppingDistance) {
            desiredVelocity = 0;
        }

        // acceleration limiting
        double velocityError =
                desiredVelocity - currentVelocity;

        double maxVelocityChange =
                maxAccel * dt;

        velocityError = Math.max(
                -maxVelocityChange,
                Math.min(maxVelocityChange, velocityError)
        );

        currentVelocity += velocityError;

        // integrate velocity
        profiledPosition += currentVelocity * dt;

        // prevent overshoot
        if (Math.signum(targetPosition - profiledPosition)
                != Math.signum(error)) {

            profiledPosition = targetPosition;
            currentVelocity = 0;
        }

        return profiledPosition;
    }

    public double getVelocity() {
        return currentVelocity;
    }

    public double getPosition() {
        return profiledPosition;
    }
}