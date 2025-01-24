from experiments import register_experiment
from experiments.base import BaseExperiment, ExperimentMetadata

@register_experiment("generated_2")
class Generated_2Experiment(BaseExperiment):
    """
    asdasd
    """
    
    def get_metadata(self) -> ExperimentMetadata:
        return ExperimentMetadata(
            name="Generated 2",
            description="""
            asdasd
            """,
            parameters={},
            datasets=['Carla/2', 'Carla/3', 'Carla/4', 'Carla/5', 'Carla/6', 'Carla/7', 'Carla/8']
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
            "Необходимо реализовать метод generate_configs для эксперимента Generated 2!"
            "\nПараметры для перебора: {}"
        )
