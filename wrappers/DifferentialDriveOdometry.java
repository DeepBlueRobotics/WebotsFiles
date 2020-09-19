public class DifferentialDriveOdometry {
    private double prevLeftMeters, prevRightMeters;
    private double poseX, poseY;
    private double heading;

    public DifferentialDriveOdometry() {
        this(0.0, 0.0, 0.0);
    }

    public DifferentialDriveOdometry(double heading) {
        this(0.0, 0.0, heading);
    }

    public DifferentialDriveOdometry(double poseX, double poseY) {
        this(poseX, poseY, 0.0);
    }

    public DifferentialDriveOdometry(double poseX, double poseY, double heading) {
        this.poseX = poseX;
        this.poseY = poseY;
        this.heading = heading;
        prevLeftMeters = 0.0;
        prevRightMeters = 0.0;
    }

    public double getX() { return poseX; }
    public double getY() { return poseY; }
    public double getHeading() { return heading; }

    public void reset(double poseX, double poseY, double heading) {
        this.poseX = poseX;
        this.poseY = poseY;
        this.heading = heading;
        prevLeftMeters = 0.0;
        prevRightMeters = 0.0;
    }

    // Update the pose of the robot using the robot heading and total meters travelled total on the left and right side
    public void update(double gyroHeading, double leftMeters, double rightMeters) {
        double angle = gyroHeading * Math.PI/180 + Math.PI/2;
        double avgDeltaMeters = 0.5 * ((leftMeters - prevLeftMeters) + (rightMeters - prevRightMeters));
        double deltaX = Math.sin(angle) * avgDeltaMeters;
        double deltaY = -Math.cos(angle) * avgDeltaMeters;
        
        poseX += deltaX;
        poseY += deltaY;
        heading = gyroHeading;
        
        prevLeftMeters = leftMeters;
        prevRightMeters = rightMeters;
    }
    
    public String toString() {
      return "(" + String.format("%.5f", poseX) + ", " + String.format("%.5f", poseY) + ", " + String.format("%.5f", heading) + ")";
    }
}