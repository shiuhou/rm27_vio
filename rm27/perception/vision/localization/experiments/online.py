"""VIO observations; missing tracking/reset evidence stays UNKNOWN."""
from dataclasses import asdict, dataclass
import json
from .common import canonical, require, number

STATE_MAP = {'OpenVINS': {}, 'SchurVINS': {}}


def normalized_state(backend, raw):
    require(backend in STATE_MAP, 'BACKEND_UNSUPPORTED', backend)
    return STATE_MAP[backend].get(str(raw), 'UNKNOWN')


@dataclass(frozen=True)
class OnlinePoseRecord:
    source_frame_id: int
    source_timestamp: dict
    backend: str
    backend_version: dict
    raw_backend_state: str
    tracking_state: str
    initialization_state: str
    validity: bool
    reason: str
    encoded_pts: str
    localization_fields: dict | None = None
    backend_timestamp: dict | None = None
    publish_timestamp: dict | None = None
    processing_completed_timestamp: dict | None = None
    observation_timestamp: dict | None = None
    processing_time_s: float | None = None
    map_id: str | None = None
    localization_epoch: int | None = None
    reset_event: dict | None = None
    relocalization_event: dict | None = None
    raw_output: dict | None = None
    backend_pose_convention: str | None = None
    schema_version: int = 1
    trajectory_variant: str = 'online'
    pose_convention: str = 'T_parent_child'
    contract: str = 'LocalizationEstimate'
    contract_complete: bool = False

    def __post_init__(self):
        require(type(self.schema_version) is int and self.schema_version == 1,
                'ONLINE_RECORD_INVALID', 'schema version')
        require(type(self.source_frame_id) is int and self.source_frame_id >= 0,
                'ONLINE_RECORD_INVALID','source frame ID')
        require(self.trajectory_variant == 'online', 'TRAJECTORY_VARIANT_MIXED','online record')
        require(self.tracking_state == normalized_state(self.backend,self.raw_backend_state),
                'ONLINE_STATE_INVALID','raw state mapping must be preserved')
        require(self.initialization_state == {'UNINITIALIZED':'UNINITIALIZED','INITIALIZING':'INITIALIZING','TRACKING':'INITIALIZED'}.get(self.tracking_state,'UNKNOWN'),
                'ONLINE_STATE_INVALID','initialization observation mapping')
        if self.localization_fields is not None:
            require(self.localization_fields.get('valid') is self.validity,
                    'ONLINE_VALIDITY_INVALID','pose validity disagrees with observation')
        require(type(self.validity) is bool and self.validity ==
                (self.tracking_state == 'TRACKING' and self.localization_fields is not None),
                'ONLINE_VALIDITY_INVALID','current tracking pose only, not flight eligibility')
        if self.processing_time_s is not None:
            number(self.processing_time_s,'processing time')
            require(self.processing_time_s >= 0,'ONLINE_TIME_INVALID','negative processing duration')
        canonical(asdict(self))

    def to_dict(self):
        return asdict(self)

    def to_json(self):
        return canonical(self.to_dict())

    @classmethod
    def from_json(cls, text):
        return cls(**json.loads(text))
