/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import frc.robot.BallTargetVisionPipeline;

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
public class Robot extends TimedRobot {
  private final Spark m_leftSpark = new Spark(0);
  private final Spark m_rightSpark = new Spark(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSpark, m_rightSpark);
  private final Joystick m_stick = new Joystick(0);
  //Pprivate final Joystick m_stick1 = new Joystick(1);

  /*private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;

  private VisionThread visionThread;
  private double centerX = 0.0;

  private final Object imgLock = new Object();

  @Override
  public void robotInit() {
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    visionThread = new VisionThread(camera, new BallTargetVisionPipeline(), pipeline -> {
        if (!pipeline.filterContoursOutput().isEmpty()) {
            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
            synchronized (imgLock) {
                centerX = r.x + (r.width / 2);
            }
        }
    });
    visionThread.start();
    super.robotInit();
  }*/

  //private boolean isAuto = false;

  @Override
  public void teleopPeriodic() {
    /*if (m_stick.getTriggerReleased()) {
      isAuto = false;
    }
    if (m_stick.getTriggerPressed()) {
      isAuto = true;
    }
    if (isAuto) {*/
      /*double centerX;
      synchronized (imgLock) {
          centerX = this.centerX;
      }
      double turn = centerX - (IMG_WIDTH / 2);
      m_robotDrive.arcadeDrive(0, turn * 0.005);*/
      //m_robotDrive.arcadeDrive(0, 0.5);
    //} else {
      m_robotDrive.arcadeDrive(-m_stick.getY(), m_stick.getX());
    //}
  }
}
