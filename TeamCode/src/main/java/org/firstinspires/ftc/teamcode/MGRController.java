package org.firstinspires.ftc.teamcode;

public class MGRController {

    private double kP;
    private double kI;
    private double kD;

    private double setpoint;
    private double integral;
    private double previousError;
    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;

    private long lastRunAt = 0;

    private boolean firstRun = true;

    public MGRController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    public void reset() {
        integral = 0.0;
        previousError = 0.0;
        firstRun = true;
    }

    private double MGRDiff(double a1, double a2) {
        final double C = 3.3/2;
        if (a2 > a1 && a2-a1 > C)       return a2 - 3.3 - a1;
        else if (a2 > a1 && a2-a1 <= C) return a2 - a1;
        else if (a1 > a2 && a1-a2 > C)  return 3.3 - a1 + a2;
        else if (a1 > a2 && a1-a2 <= C) return a2 - a1;
        else return 0;
    }

    public double calculate(double measurement) {
        if (0 == lastRunAt)
            lastRunAt = System.currentTimeMillis();
        double dtSeconds = (System.currentTimeMillis() - lastRunAt) / 1000.0;
        lastRunAt = System.currentTimeMillis();
        double error = MGRDiff(setpoint, measurement);

        // Proportional term
        double pTerm = kP * error;

        // Integral term
        integral += error * dtSeconds;
        double iTerm = kI * integral;

        // Derivative term
        double derivative;
        if (firstRun) {
            derivative = 0.0;
            firstRun = false;
        } else {
            derivative = (error - previousError) * (1.0 / dtSeconds);
        }
        double dTerm = kD * derivative;

        double output = pTerm + iTerm + dTerm;

        // Clamp output
        if (output > outputMax) {
            output = outputMax;
        } else if (output < outputMin) {
            output = outputMin;
        }

        previousError = error;
        return output;
    }

    // Optional: getters/setters for gains
    public void setP(double kP) { this.kP = kP; }
    public void setI(double kI) { this.kI = kI; }
    public void setD(double kD) { this.kD = kD; }

    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }
}

