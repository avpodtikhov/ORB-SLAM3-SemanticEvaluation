from experiments import register_experiment
from experiments.base import BaseExperiment, ExperimentMetadata

@register_experiment("generated")
class generatedExperiment(BaseExperiment):
    """
    Автоматически сгенерированный эксперимент
    """
    
    def get_metadata(self) -> ExperimentMetadata:
        return ExperimentMetadata(
            name="generated",
            description="""
            Автоматически сгенерированный эксперимент
            """,
            parameters={},
            datasets=['Carla/4']
        )
    
    def generate_configs(self) -> None:
        """
        Генерация YAML конфигураций для эксперимента.
        
        Параметры эксперимента:
        Нет параметров
        
        Пример использования:
            config = self.base_config.copy()
            config.update({
                "ORBextractor": {
                    "param1": value1,
                    "param2": value2
                }
            })
            self.save_config(config, f"config_name")
        """
        raise NotImplementedError(
            "Необходимо реализовать метод generate_configs для эксперимента generated!"
            "\nПараметры для перебора: {}"
        )
