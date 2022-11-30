// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.vision.AprilTag;
import frc.robot.vision.AprilTagFieldLayout;

public class Robot extends TimedRobot {

    // field2d visualization
    Field2d field2d = new Field2d();
    // for visualization, use a quarter-sized field
    final double kFieldLength = Units.feetToMeters(54/2.0);
    final double kFieldWidth = Units.feetToMeters(27/2.0);
    // transform the estimated layout to be in the middle of the vizualized field
    final Pose3d kVizOrigin = new Pose3d(
        kFieldLength/2.0,
        kFieldWidth/2.0,
        0,
        new Rotation3d(
            0,
            0,
            Math.PI
        )
    );

    PhotonCamera camera;
    TagLayoutId layoutId = new TagLayoutId();
    AprilTagFieldLayout estLayout = new AprilTagFieldLayout(new ArrayList<>(), kFieldLength, kFieldWidth);
    
    @Override
    public void robotInit() {
        var instance = NetworkTableInstance.getDefault();
        if(isSimulation()) {
            instance.stopServer();
            // set the NT server if simulating this code.
            // "localhost" for photon on desktop, or "photonvision.local" / "[ip-address]" for coprocessor
            instance.setServer("photonvision.local");
            instance.startClient3("myRobot");
        }

        camera = new PhotonCamera(instance, "cameraName"); // match the camera name in the dashboard
        SmartDashboard.putData("Layout Identification Viz", field2d);
        // button to serialize the current estimated layout to JSON
        SmartDashboard.putData("Serialize JSON", Commands.runOnce(()->{
            try {
                estLayout.serialize("estimatedTagLayout.json");
            } catch(Exception e) {
                e.printStackTrace();
            }
        }).ignoringDisable(true));
    }

    @Override
    public void robotPeriodic() {
        var result = camera.getLatestResult();
        var visibleTags = new ArrayList<AprilTag>();
        var robotPoses = new ArrayList<Pose3d>();
        for(var target : result.getTargets()) {
            var camToTarg = target.getBestCameraToTarget();
            // currently visible tags to update the estimated layout with
            visibleTags.add(new AprilTag(
                target.getFiducialId(),
                new Pose3d().transformBy(camToTarg)
            ));
            // the best estimation of the robot pose on the field relative to this tag
            var tagPose = estLayout.getTagPose(target.getFiducialId());
            if(tagPose.isPresent()) {
                robotPoses.add(tagPose.get().transformBy(camToTarg.inverse()));
            }
        }

        layoutId.update(visibleTags);
        estLayout = new AprilTagFieldLayout(layoutId.findLayout(), kFieldLength, kFieldWidth);
        // visualize current tags in estimated layout
        field2d.getObject("estLayout").setPoses(
            estLayout.getTags().stream()
                .map(t -> t.pose.relativeTo(kVizOrigin).toPose2d())
                .collect(Collectors.toList())
        );
        // visualize robot poses relative to current estimated layout
        field2d.getObject("robotPoses").setPoses(
            robotPoses.stream()
                .map(p -> p.relativeTo(kVizOrigin).toPose2d())
                .collect(Collectors.toList())
        );
    }
}
