/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Spark m_leftSpark = new Spark(0); // Left motor controller
  private final Spark m_rightSpark = new Spark(1); // Right motor controller
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSpark, m_rightSpark); // Combined motor control
  private final Joystick m_stick = new Joystick(0); // Primary joystick/controller
  private Ultrasonic ultrasonic = new Ultrasonic(1, 0); // Ultrasonic sensor

  private static final int IMG_WIDTH = 160; // Vision image width(pixels)
  private static final int IMG_HEIGHT = 120; // Vision image height(pixels)

  private VisionThread visionThread; // Java Thread class housing vision processing callback (init below)
  private double centerX = 1024.0; // Stores the current center of target; 1024.0 signifies "No Target"

  private final Object imgLock = new Object(); // Syncronizes read/write calls from/to centerX

  private final double visionPrecision = 0.05; // The lower this value, the more precise vision is. The higher the value, the less the robot "jitters" trying to find the "perfect" center
  private final double visionMinimumSpeed = 0.5; // The minimum speed at which the motors will start moving.

  /* Initializes the robot */
  @Override
  public void robotInit() {
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(); // Starts capture from camera, streams to dashboard and vision code
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT); // Sets camera resolution
    camera.setExposureManual(20); // Sets camera exposure low for more reliable vision processing

    visionThread = new VisionThread(camera, new BallTargetVisionPipeline(), pipeline -> { // Initialize visionthread
      if (!pipeline.filterContoursOutput().isEmpty()) { // Were any contours found by vision pipeline?
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0)); // If so, get the bounding Rect of the first contour
        synchronized (imgLock) { // Syncronizes read/write calls from/to centerX
            centerX = r.x + (r.width / 2); // Sets centerX to center position of contour
        }
      } else { // No contours found
        synchronized (imgLock) { // Syncronizes read/write calls from/to centerX
          centerX = 1024; // 1024 signifies "No Target"
        }
      }
    });
    visionThread.start(); // Starts vision thread
    ultrasonic.setAutomaticMode(true); // Sets ultrasonic sensor to "round-robin" mode - automatically updates all sensors
    super.robotInit(); // Initialize robot
  }

  /* Handles autonomous mode */
  @Override
  public void autonomousPeriodic() {
    double centerX; // Storage for centerX value
    synchronized (imgLock) { // Syncronizes read/write calls from/to centerX
        centerX = this.centerX; // Sets centerX to saved centerX
    }
    if (centerX == 1024) { // No countors were found
      m_robotDrive.arcadeDrive(0, visionMinimumSpeed); // Spin clockwise
    } else {
      double turn = centerX - (IMG_WIDTH / 2); // Countour found, convert centerX into a value which is negative if on the left or positive on right
      turn *= 0.004; // Scales down centerX to fit within {-1, 1} range
      if (Math.abs(turn) <= visionPrecision) { // If target is close enough to precise center
        turn = 0; // Don't turn
      } else if (turn < 0 && turn >= -visionMinimumSpeed) { // If target is not close enough to center but motor speed will not activate motors
        turn = -visionMinimumSpeed; // Set the motor speed to minimum value so that the robot actually turns
      } else if (turn <= visionMinimumSpeed) { // If target is not close enough to center but motor speed will not activate motors
        turn = visionMinimumSpeed; // Set the motor speed to minimum value so that the robot actually turns
      }
      System.out.println(turn); // Debug
      m_robotDrive.arcadeDrive(0, turn); // Rotate the robot in place
    }
  }
  
  @Override
  public void teleopPeriodic() {
    if (m_stick.getRawButton(5)) {
      SmartDashboard.putString("DB/String 0", "Locating Target...");
      double centerX;
      synchronized (imgLock) {
          centerX = this.centerX;
      }
      if (centerX != 1024) {
        double turn = centerX - (IMG_WIDTH / 2);
        turn *= 0.004;
        if (Math.abs(turn) <= visionPrecision) {
          turn = 0;
        } else if (turn < 0 && turn >= -visionMinimumSpeed) {
          turn = -visionMinimumSpeed;
        } else if (turn <= visionMinimumSpeed) {
          turn = visionMinimumSpeed;
        }
        m_robotDrive.arcadeDrive(0, turn);
      }
    } else {
      SmartDashboard.putString("DB/String 0", "Vision inactive.");
      m_robotDrive.arcadeDrive(m_stick.getY()*0.75, m_stick.getX()*0.75);
    }
    System.out.print(ultrasonic.getRangeMM());
    System.out.print(" MM ; ");
    System.out.print(ultrasonic.getRangeInches());
    System.out.println(" IN");
  }
}
