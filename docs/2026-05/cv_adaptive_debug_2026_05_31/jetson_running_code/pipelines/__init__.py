from avros_perception.pipelines.adaptive import AdaptivePipeline
from avros_perception.pipelines.base import Pipeline, PipelineResult
from avros_perception.pipelines.hsv import HSVPipeline
from avros_perception.pipelines.sooner25 import Sooner25Pipeline
from avros_perception.pipelines.stub import StubPipeline

__all__ = [
    'Pipeline',
    'PipelineResult',
    'StubPipeline',
    'HSVPipeline',
    'Sooner25Pipeline',
    'AdaptivePipeline',
    'PIPELINES',
    'build_pipeline',
]

PIPELINES = {
    'stub': StubPipeline,
    'hsv': HSVPipeline,
    'sooner25': Sooner25Pipeline,
    'adaptive': AdaptivePipeline,
}


def build_pipeline(name: str, params: dict, logger=None) -> Pipeline:
    if name not in PIPELINES:
        raise ValueError(
            f"Unknown pipeline '{name}'. Available: {sorted(PIPELINES)}"
        )
    return PIPELINES[name](params, logger)
