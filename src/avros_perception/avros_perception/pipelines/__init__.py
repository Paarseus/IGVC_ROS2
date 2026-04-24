from avros_perception.pipelines.base import Pipeline, PipelineResult
from avros_perception.pipelines.stub import StubPipeline

__all__ = ['Pipeline', 'PipelineResult', 'StubPipeline', 'PIPELINES', 'build_pipeline']

PIPELINES = {
    'stub': StubPipeline,
}


def build_pipeline(name: str, params: dict, logger=None) -> Pipeline:
    if name not in PIPELINES:
        raise ValueError(
            f"Unknown pipeline '{name}'. Available: {sorted(PIPELINES)}"
        )
    return PIPELINES[name](params, logger)
