from pydantic import BaseModel

class TwistLinearVelocity(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

class TwistAngularVelocity(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

class ROSTwist(BaseModel):
    linear: TwistLinearVelocity
    angular: TwistAngularVelocity

class Velocity2D(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    vth: float = 0.0


