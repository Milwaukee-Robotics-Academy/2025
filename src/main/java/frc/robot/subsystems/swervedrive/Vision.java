package frc.robot.subsystems.swervedrive;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveDrive;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
  private final PhotonCamera m_leftCammera;
  private PhotonCamera m_rightCammera;
  private final PhotonPoseEstimator photonEstimatorLeft;
  private final PhotonPoseEstimator photonEstimatorRight;
  private Matrix<N3, N1> curStdDevs;

    public Vision() {
      m_leftCammera = new PhotonCamera(kCameraNameLeft);
      m_rightCammera = new PhotonCamera(kCameraNameRight);
    photonEstimatorLeft = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamLeft);
    photonEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonEstimatorRight = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamRight);
    photonEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    SmartDashboard.putBoolean("Use Vision?", true);
  }

//  public Vision(Object object, Field2d field) {
      //TODO Auto-generated constructor stub
  //  }

  public void updatePoseEstimation(SwerveDrive drive) {

    if (SmartDashboard.getBoolean("Use Vision?", false)) {
      // Correct pose estimate with vision measurements
    //  processVisionEstimation(this.getEstimatedGlobalPoseFromLeft(), drive, "VisionLeft");
      processVisionEstimation(this.getEstimatedGlobalPoseFromRight(), drive, "VisionRight");
    }
  }

  private void processVisionEstimation(Optional<EstimatedRobotPose> poseEst, SwerveDrive drive, String visionLabel) {
    poseEst.ifPresent(est -> {
      // Change our trust in the measurement based on the tags we can see
      var estStdDevs = this.getEstimationStdDevs();

      // Checks if the values from vision are within field limits
      if ((est.estimatedPose.getTranslation().getY() > -0.05 && est.estimatedPose.getTranslation().getY() < 8.35) &&
        (est.estimatedPose.getTranslation().getX() > -0.05 && est.estimatedPose.getTranslation().getX() < 20.0)) {
        drive.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      }
      SmartDashboard.putNumber(visionLabel + "/x", est.estimatedPose.getTranslation().getX());
      SmartDashboard.putNumber(visionLabel + "/y", est.estimatedPose.getTranslation().getY());
      SmartDashboard.putNumber(visionLabel + "/angle", est.estimatedPose.toPose2d().getRotation().getDegrees());
      SmartDashboard.putNumber(visionLabel + "/posetimestamp", est.timestampSeconds);
    });
  }
  

  /**
   * The latest estimated robot pose on the field from vision data. This may be
   * empty. This should
   * only be called once per loop.
   *
   * <p>
   * Also includes updates for the standard deviations, which can (optionally) be
   * retrieved with
   * {@link getEstimationStdDevs}
   *
   * @param camera The PhotonCamera to get the results from.
   * @param estimator The PhotonPoseEstimator to update the pose.
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
   *         timestamp, and targets
   *         used for estimation.
   */
  private Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonCamera camera, PhotonPoseEstimator estimator) {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : camera.getAllUnreadResults()) {
      visionEst = estimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());
    }
    return visionEst;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFromLeft() {
    return getEstimatedGlobalPose(m_leftCammera, photonEstimatorLeft);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFromRight() {
    return getEstimatedGlobalPose(m_rightCammera, photonEstimatorRight);
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates
   * dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from
   * the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets       All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimatorLeft.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

}