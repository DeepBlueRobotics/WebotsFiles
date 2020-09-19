import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Joystick;
import com.cyberbotics.webots.controller.Gyro;

public class DifferentialDrivetrain {
    
  private static final double inchToMeter = 0.0254;

  // Measured in miliseconds
  private static final int samplingRate = 20;

  private static final double MAX_JOY = 32768.0;
  private static final double JOY_THRESHOLD = 0.02;
  
  private static final double wheelBase = 18.15 * inchToMeter;
  private static final double trackWidth = 17.75 * inchToMeter;
  
  // Radians per second
  private static final double CIMFreeSpeed = 5330 * (2 * Math.PI) / 60.0;
  private static final double driveGearing = 6.67;
  private static final double wheelDiameter = 5.0 * inchToMeter;
  // Radius * Free Speed / Gear Ratio
  private static final double maxSpeed = (wheelDiameter / 2.0) * CIMFreeSpeed / driveGearing;
  // Treat robot as point mass moving around in a circle
  // Tangential speed / "radius"
  private static final double maxRotation = maxSpeed / Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));
  private static final double maxAccel = 9.8;

  private static final double encoderCPR = 5.0;
  private static final double distancePerTick = (Math.PI * wheelDiameter) / (encoderCPR * driveGearing);

  private static final String motorNames[] = {"Motor FL", "Motor FR", "Motor BL", "Motor BR"};
  private static final String encoderNames[] = {"Encoder FL", "Encoder FR"};
  private static final String gyroName = "Gyro";
  
  private static double prevLeft = 0.0;
  private static double prevRight = 0.0;
  
  private static DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(0.0, 0.0, Math.PI/2);

  private static boolean slowMode = false;

  public static void main(String[] args) {

    // Initialize the robot and the initial timestep
    Robot robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Initialize the motors to default settings
    Motor motors[] = new Motor[motorNames.length];
    for (int i = 0; i < motorNames.length; i++) {
      motors[i] = robot.getMotor(motorNames[i]);
      motors[i].setPosition(Double.POSITIVE_INFINITY);
      motors[i].setVelocity(0.0);
    }
    
    // Enable the encoders
    PositionSensor encoderFL = new PositionSensor(encoderNames[0]);
    PositionSensor encoderFR = new PositionSensor(encoderNames[1]);
    encoderFL.enable(samplingRate);
    encoderFR.enable(samplingRate);
    
    // Enable the gyro
    Gyro gyro = new Gyro(gyroName);
    gyro.enable(samplingRate);
    
    // Enable the joystick
    Joystick joystick = robot.getJoystick();
    joystick.enable(samplingRate);
    
    // Variables to help determine speed
    double motorVals[] = new double[2];
    double speed, rotation, left, right;
    double leftMeters, rightMeters, gyroHeading;
    double leftAccel, rightAccel;
    double dt = 0.032;

    while (robot.step(timeStep) != -1) {
      speed = -joystick.getAxisValue(2) / MAX_JOY;
      rotation = -joystick.getAxisValue(3) / MAX_JOY;
      
      // Square the joysticks
      speed = Math.copySign(speed * speed, speed);
      rotation = Math.copySign(rotation * rotation, rotation);
      
      // If the speed is at a small value like 0.02, just round down to zero
      if (Math.abs(speed) < JOY_THRESHOLD) speed = 0.0;
      if (Math.abs(rotation) < JOY_THRESHOLD) rotation = 0.0;
      
      // Toggle slow mode
      if (joystick.getPressedButton() == 1) slowMode = slowMode ? false : true;
      
      // Arcade drive + slow mode
      motorVals = arcadeDrive(speed, rotation);
      left = motorVals[0] * (slowMode ? 0.5 : 1);
      right = motorVals[1] * (slowMode ? 0.5 : 1);
      
      // Limit maximum acceleration on the left side
      leftAccel = maxSpeed * (left - prevLeft) / dt;
      leftAccel = Math.copySign(Math.min(leftAccel, maxAccel), leftAccel);
      left = (leftAccel * dt + prevLeft) / maxSpeed;
      
      // Limit maximum acceleration on the right side
      rightAccel = maxSpeed * (right - prevRight) / dt;
      rightAccel = Math.copySign(Math.min(rightAccel, maxAccel), rightAccel);
      right = (rightAccel * dt + prevRight) / maxSpeed;
      
      // Set the motor speeds
      motors[0].setVelocity(left * CIMFreeSpeed);
      motors[1].setVelocity(right * CIMFreeSpeed);
      motors[2].setVelocity(left * CIMFreeSpeed);
      motors[3].setVelocity(right * CIMFreeSpeed);
      
      // Get the total distance travelled by the encoders on each side
      leftMeters = encoderFL.getValue() * distancePerTick;
      rightMeters = encoderFR.getValue() * distancePerTick;
      
      // Convert gyro reading from angular velocity about the y-axis to degrees
      gyroHeading = odometry.getHeading() + (gyro.getValues()[1] * dt * 180 / Math.PI);
      gyroHeading = gyroHeading % 360;
      gyroHeading = (gyroHeading < 0) ? 360 + gyroHeading : gyroHeading;
      
      // Update and print the robot pose
      odometry.update(gyroHeading, leftMeters, rightMeters);
      System.out.println(robot.getTime() + ", Pose = " + odometry.toString());
    
      prevLeft = left;
      prevRight = right;
    }
  }
  
  // Arcade drive: (speed, rotation) ==> (left, right) 
  public static double[] arcadeDrive(double speed, double rotation) {
    double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);
    double left, right;
    
    left = (speed - rotation) * 0.5;
    right = (speed + rotation) * 0.5;
    
    // Clamp the speeds
    left = Math.copySign(Math.min(Math.abs(left), 1.0), left);
    right = Math.copySign(Math.min(Math.abs(right), 1.0), right);
    
    return new double[]{left, right};
  }
}
