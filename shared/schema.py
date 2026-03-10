from __future__ import annotations

from typing import Dict, List

from pydantic import BaseModel, Field


class Observation(BaseModel):
    # Preferred observation field for robot proprioception.
    joint_state: List[float] = Field(default_factory=list)
    # Optional backward-compat field from the initial minimal prototype.
    state: List[float] = Field(default_factory=list)
    image_b64: str | None = None
    image_b64_map: Dict[str, str] | None = None
    instruction: str | None = None
    metadata: Dict[str, str] = Field(default_factory=dict)

    class Config:
        extra = "ignore"

    def policy_state(self) -> List[float]:
        if self.joint_state:
            return self.joint_state
        return self.state


class Action(BaseModel):
    action: List[float]


def model_dump_compat(model: BaseModel) -> dict:
    if hasattr(model, "model_dump"):
        return model.model_dump()
    return model.dict()
