/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
//import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  private final Spark m_leftSpark = new Spark(0); // Left motor controller
  private final Spark m_rightSpark = new Spark(1); // Right motor controller
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSpark, m_rightSpark); // Combined motor
                                                                                                   // control
  private final Joystick m_stick = new Joystick(0); // Primary joystick/controller
  //private Ultrasonic ultrasonic = new Ultrasonic(1, 0); // Ultrasonic sensor

  private static final int IMG_WIDTH = 160; // Vision image width(pixels)
  private static final int IMG_HEIGHT = 120; // Vision image height(pixels)

  private VisionThread visionThread; // Java Thread class housing vision processing callback (init below)
  private Point targetCenter = new Point(1024, 1024); // Stores the current center of target; (1024,1024) signifies "No Target"

  private final Object imgLock = new Object(); // Syncronizes read/write calls from/to targetCenter

  private final double visionPrecision = 0.035; // The lower this value, the more precise vision is. The higher the
                                                // value, the less the robot "jitters" trying to find the "perfect"
                                                // center
  private final double visionMinimumSpeed = 0.5; // The minimum speed at which the motors will start moving.

  private CvSource aimVideoSource; // Custom video feed sent to dashboard (shows R/Y/G indication)

  /* Initializes the robot */
  @Override
  public void robotInit() {
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(); // Starts capture from camera, streams to
                                                                           // dashboard and vision code
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT); // Sets camera resolution
    
    camera.setExposureManual(20); // Sets camera exposure low for more reliable vision processing
    camera.setBrightness(20);
    camera.setConfigJson("{\"properties\": [{\"name\": \"raw_exposure_absolute\", \"value\": 20}]}");
    System.out.println(camera.getConfigJson());
    aimVideoSource = CameraServer.getInstance().putVideo("Aim Assistance", IMG_WIDTH, IMG_HEIGHT);
    
    final int crosshairSize = 10;
    
    visionThread = new VisionThread(camera, new BallTargetVisionPipeline(), pipeline -> { // Initialize visionthread
      boolean rumble = false;
      Mat aimMat = pipeline.resizeImageOutput(); // Mat object from camera for Aim Assistance
      if (!pipeline.filterContoursOutput().isEmpty()) { // Were any contours found by vision pipeline?
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0)); // If so, get the bounding Rect of the first contour
        synchronized (imgLock) { //Syncronizes read/write calls from/to targetCenter
          targetCenter = new Point(r.x + (r.width / 2), r.y + (r.height / 2)); // Sets targetCenter to center position of contour
          Imgproc.drawMarker(aimMat, new Point(IMG_WIDTH/2, IMG_HEIGHT/2), new Scalar(255, 255, 255), Imgproc.MARKER_CROSS, crosshairSize);
          final double centerPrecision = 10;

          if (targetCenter.x >= (IMG_WIDTH/2)-centerPrecision && targetCenter.x <= (IMG_WIDTH/2)+centerPrecision/* && targetCenter.y >= (IMG_HEIGHT/2)-centerPrecision && targetCenter.y <= (IMG_HEIGHT/2)+centerPrecision*/) {
            Imgproc.drawMarker(aimMat, targetCenter, new Scalar(0, 255, 0), Imgproc.MARKER_CROSS, crosshairSize);
            rumble = true;
          } else {
            Imgproc.drawMarker(aimMat, targetCenter, new Scalar(0, 255, 255), Imgproc.MARKER_CROSS, crosshairSize);
          }
        }
      } else { // No contours found
        synchronized (imgLock) { // Syncronizes read/write calls from/to targetCenter
          targetCenter = new Point(1024, 1024); // 1024 signifies "No Target"
          Imgproc.drawMarker(aimMat, new Point((IMG_WIDTH/2), (IMG_HEIGHT/2)), new Scalar(0, 0, 255), Imgproc.MARKER_CROSS, crosshairSize);
        }
      }
      aimVideoSource.putFrame(aimMat); // Publish Aim Assistance to dashboard
      if (rumble) {
        m_stick.setRumble(RumbleType.kLeftRumble, 1);
        m_stick.setRumble(RumbleType.kRightRumble, 1);
      } else {
        m_stick.setRumble(RumbleType.kLeftRumble, 0);
        m_stick.setRumble(RumbleType.kRightRumble, 0);
      }
    });
    visionThread.start(); // Starts vision thread
    //ultrasonic.setAutomaticMode(true); // Sets ultrasonic sensor to "round-robin" mode - automatically updates all sensors
    super.robotInit(); // Initialize robot
  }

  /* Handles autonomous mode */
  @Override
  public void autonomousPeriodic() {
    Point targetCenter; // Storage for centerX value
    synchronized (imgLock) { // Syncronizes read/write calls from/to centerX
      targetCenter = this.targetCenter; // Sets centerX to saved centerX
    }
    if (targetCenter.x == 1024) { // No countors were found
      m_robotDrive.arcadeDrive(0, visionMinimumSpeed); // Spin clockwise
    } else {
      double turn = targetCenter.x - (IMG_WIDTH / 2); // Countour found, convert centerX into a value which is negative if on the left or positive on right
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
    if (m_stick.getRawButton(6)) {
      Point targetCenter;
      synchronized (imgLock) {
        targetCenter = this.targetCenter;
      }
      if (targetCenter.x == 1024) {
        m_robotDrive.arcadeDrive(0, 0);
      } else {
        double turn = targetCenter.x  - (IMG_WIDTH / 2);
        turn *= 0.008;
        System.out.print("Turn (pre-limit): ");
        System.out.print(turn);
        if (Math.abs(turn) <= visionPrecision) {
          turn = 0;
        } else if (turn < 0 && turn >= -visionMinimumSpeed) {
          turn = -visionMinimumSpeed;
        } else if (turn > 0 && turn <= visionMinimumSpeed) {
          turn = visionMinimumSpeed;
        }
        System.out.print("; Turn (post-limit): ");
        System.out.println(turn);
        m_robotDrive.arcadeDrive(0, turn);
      }
    } else {
      if (m_stick.getPOV() == 0) {
        m_robotDrive.arcadeDrive(-0.5, 0);
      } else if (m_stick.getPOV() == 90) {
        m_robotDrive.arcadeDrive(0, 0.5);
      } else if (m_stick.getPOV() == 180) {
        m_robotDrive.arcadeDrive(0.5, 0);
      } else if (m_stick.getPOV() == 270) {
        m_robotDrive.arcadeDrive(0, -0.5);
      } else {
        m_robotDrive.arcadeDrive(m_stick.getY()*0.75, m_stick.getX()*0.75);
      }
    }
    /*System.out.print(ultrasonic.getRangeMM());
    System.out.print(" MM ; ");
    System.out.print(ultrasonic.getRangeInches());
    System.out.println(" IN");*/
  }
}
