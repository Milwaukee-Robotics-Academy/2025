package frc.robot.subsystems.swervedrive;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
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
  private final PhotonCamera leftCammera;
  private PhotonCamera rightCammera;
  private final PhotonPoseEstimator photonEstimatorLeft;
  private final PhotonPoseEstimator photonEstimatorRight;
  private Matrix<N3, N1> curStdDevs;

  
    public Vision() {
      leftCammera = new PhotonCamera(kCameraNameLeft);
      rightCammera = new PhotonCamera(kCameraNameRight);
    photonEstimatorLeft = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamLeft);
    photonEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonEstimatorRight = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamRight);
    photonEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public void updatePoseEstimation(SwerveDrive drive) {
    // Correct pose estimate with vision measurements
    var poseEstLeft = this.getEstimatedGlobalPoseFromLeft();

    poseEstLeft.ifPresent(
        est -> {
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = this.getEstimationStdDevs();

          drive.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          SmartDashboard.putNumber("VisionLeft/x", est.estimatedPose.getTranslation().getX());
          SmartDashboard.putNumber("VisionLeft/y", est.estimatedPose.getTranslation().getY());
          SmartDashboard.putNumber("VisionLeft/angle", est.estimatedPose.toPose2d().getRotation().getDegrees());
          SmartDashboard.putNumber("VisionLeft/time", Timer.getFPGATimestamp());
          SmartDashboard.putNumber("VisionLeft/posetimestamp", est.timestampSeconds);
          SmartDashboard.putBoolean("VisionLeft/CurrentPose",true);
        });
    // if(poseEstLeft.isEmpty()) {
    //             SmartDashboard.putBoolean("VisionLeft/CurrentPose", false);
    // }
    var poseEstRight = this.getEstimatedGlobalPoseFromRight();

    poseEstRight.ifPresent(
        est -> { 
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = this.getEstimationStdDevs();

          drive.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          SmartDashboard.putNumber("VisionRight/x", est.estimatedPose.getTranslation().getX());
          SmartDashboard.putNumber("VisionRight/y", est.estimatedPose.getTranslation().getY());
          SmartDashboard.putNumber("VisionRight/angle", est.estimatedPose.toPose2d().getRotation().getDegrees());
          SmartDashboard.putNumber("VisionRight/time", Timer.getFPGATimestamp());
          SmartDashboard.putNumber("VisionRight/posetimestamp", est.timestampSeconds);
          SmartDashboard.putBoolean("VisionRight/CurrentPose",true);
        });
    //if(poseEstRight.isEmpty()) {
      //          SmartDashboard.putBoolean("VisionRight/CurrentPose", false);
    //}
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
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
   *         timestamp, and targets
   *         used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFromLeft() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : leftCammera.getAllUnreadResults()) {
      visionEst = photonEstimatorLeft.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());

    }
    return visionEst;
  }

   public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFromRight() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : rightCammera.getAllUnreadResults()) {
      visionEst = photonEstimatorRight.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());

    }
    return visionEst;
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