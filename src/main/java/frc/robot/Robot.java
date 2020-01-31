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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
public class Robot extends TimedRobot {
  private final Spark m_leftSpark = new Spark(0);
  private final Spark m_rightSpark = new Spark(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSpark, m_rightSpark);
  private final Joystick m_stick = new Joystick(0);

  private static final int IMG_WIDTH = 160;
  private static final int IMG_HEIGHT = 120;

  private VisionThread visionThread;
  private double centerX = 1024.0;

  private final Object imgLock = new Object();

  @Override
  public void robotInit() {
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    camera.setExposureManual(20);

    visionThread = new VisionThread(camera, new BallTargetVisionPipeline(), pipeline -> {
      if (!pipeline.filterContoursOutput().isEmpty()) {
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        synchronized (imgLock) {
            centerX = r.x + (r.width / 2);
        }
      } else {
        synchronized (imgLock) {
          centerX = 1024;
        }
      }
    });
    visionThread.start();
    m_robotDrive.setSafetyEnabled(false);
    super.robotInit();
  }

  @Override
  public void autonomousPeriodic() {
    double centerX;
    synchronized (imgLock) {
        centerX = this.centerX;
    }
    System.out.println(centerX);
    if (centerX == 1024) {
      m_robotDrive.arcadeDrive(0, 0.5);
    } else {
      double turn = centerX - (IMG_WIDTH / 2);
      turn *= 0.004;
      if (Math.abs(turn) <= 0.07) {
        turn = 0;
      } else if (turn < 0 && turn >= -0.5) {
        turn = -0.5;
      } else if (turn <= 0.5) {
        turn = 0.5;
      }
      m_robotDrive.arcadeDrive(0, turn);
    }
  }
  
  @Override
  public void teleopPeriodic() {
    synchronized (imgLock) {
      System.out.println(this.centerX);
    }
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }
}
