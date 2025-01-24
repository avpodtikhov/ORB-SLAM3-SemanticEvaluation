from .. import register_experiment
from ..base import BaseExperiment, ExperimentMetadata

@register_experiment("test_experiment")
class TestExperiment(BaseExperiment):
    """Простой запуск детерменированного ORB-SLAM3"""
    
    def get_metadata(self) -> ExperimentMetadata:
        return ExperimentMetadata(
            name="Тестовый эксперимент",
            description="""
            Простой запуск детерменированного ORB-SLAM3
            """,
            parameters={},
            datasets=["4"]
        )
    
    def generate_configs(self) -> None:
        config = self.base_config.copy()
        name = f"test"
        self.save_config(config, name)