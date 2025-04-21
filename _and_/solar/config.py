from typing import Dict, Any
from dataclasses import dataclass, field

@dataclass
class APIConfig:
    HOST: str = "125.131.105.165"
    PORT: int = 10086
    DEBUG: bool = True

@dataclass
class Config:
    api: APIConfig = APIConfig()
    

config = Config() 