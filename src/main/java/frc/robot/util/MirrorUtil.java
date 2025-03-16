// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466
 
package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.CoralObjective;
import frc.robot.subsystems.drive.trajectory.TrajectoryGenerationHelpers;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

@ExtensionMethod({TrajectoryGenerationHelpers.class})
public class MirrorUtil {
  @Setter @Getter private static BooleanSupplier mirror;

  public static CoralObjective apply(CoralObjective coralObjective) {
    if (!mirror.getAsBoolean()) return coralObjective;
    int shiftedBranchId = coralObjective.branchId() - 1;
    if (shiftedBranchId == -1) {
      shiftedBranchId = 11;
    }
    int flippedBranchId = 11 - shiftedBranchId;
    flippedBranchId = (++flippedBranchId == 12) ? 0 : flippedBranchId;
    return new CoralObjective(flippedBranchId, coralObjective.reefLevel());
  }

  public static Pose2d apply(Pose2d pose) {
    if (!mirror.getAsBoolean()) return pose;
    return new Pose2d(
        pose.getX(),
        FieldConstants.fieldWidth - pose.getY(),
        new Rotation2d(pose.getRotation().getCos(), -pose.getRotation().getSin()));
  }

  public static VehicleState apply(VehicleState state) {
    if (!mirror.getAsBoolean()) return state;
    Pose2d pose = apply(state.getPose());
    return VehicleState.newBuilder()
        .setX(pose.getX())
        .setY(pose.getY())
        .setTheta(pose.getRotation().getRadians())
        .setVx(state.getVx())
        .setVy(-state.getVy())
        .setOmega(-state.getOmega())
        .addAllModuleForces(
            state.getModuleForcesList().stream()
                .map(
                    forces ->
                        ModuleForce.newBuilder()
                            .setFx(forces.getFx())
                            .setFy(-forces.getFy())
                            .build())
                .toList())
        .build();
  }
}
