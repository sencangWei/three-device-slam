"""Named-frame spatial alignment primitives.

All transforms follow ``T_A_B``: a point expressed in frame B is mapped into
frame A.  The package is deliberately independent from ROS so the coordinate
and timestamp contract can be verified deterministically first.
"""

from .alignment import (
    PoseSample,
    RelativePoseObservation,
    WorldInitialization,
    body_pose_from_camera_pose,
    estimate_world_vio_anchor,
    initialize_ego_world,
    propagate_world_trajectory,
)
from .apriltag_alignment import (
    AnchorAcceptance,
    AnchorReconciliation,
    RejectedTagObservation,
    TagAnchorEstimate,
    TagMount,
    TagObservationPolicy,
    TagPoseObservation,
    estimate_tag_anchor_window,
    reconcile_anchor,
    tag_observation_to_camera_gripper,
)
from .apriltag_detector import (
    AprilTagDetectionBatch,
    AprilTagDetectorConfig,
    AprilTagImageDetection,
    CameraCalibration,
    PnPCandidate,
    PnPSelection,
    detect_apriltags,
    select_ippe_candidate,
)
from .tag_world_constraints import (
    GridPose,
    TagWorldConstraint,
    aggregate_transform,
    analyze_fixed_transform_stability,
    aprilgrid_object_corners,
    make_world_constraint,
    solve_grid_pose,
)

__all__ = [
    "PoseSample",
    "RelativePoseObservation",
    "WorldInitialization",
    "AnchorAcceptance",
    "AnchorReconciliation",
    "RejectedTagObservation",
    "TagAnchorEstimate",
    "TagMount",
    "TagObservationPolicy",
    "TagPoseObservation",
    "AprilTagDetectionBatch",
    "AprilTagDetectorConfig",
    "AprilTagImageDetection",
    "CameraCalibration",
    "PnPCandidate",
    "PnPSelection",
    "GridPose",
    "TagWorldConstraint",
    "aggregate_transform",
    "analyze_fixed_transform_stability",
    "aprilgrid_object_corners",
    "body_pose_from_camera_pose",
    "detect_apriltags",
    "estimate_tag_anchor_window",
    "estimate_world_vio_anchor",
    "initialize_ego_world",
    "make_world_constraint",
    "propagate_world_trajectory",
    "reconcile_anchor",
    "select_ippe_candidate",
    "solve_grid_pose",
    "tag_observation_to_camera_gripper",
]
