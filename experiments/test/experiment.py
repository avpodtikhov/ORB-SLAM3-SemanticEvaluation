from experiments import register_experiment
from experiments.base import BaseExperiment, ExperimentMetadata
from typing import List

@register_experiment("test")
class TestExperiment(BaseExperiment):
    """
    test
    """
    
    def get_metadata(self) -> ExperimentMetadata:
        return ExperimentMetadata(
            name="test",
            description="""
            test
            """,
            default_slam_tag="1.0.0",
            datasets=['Carla/4'],
            parameters={}
        )
    
    def generate_configs(self) -> List[str]:
        """
        Генерация YAML конфигураций для эксперимента.
        
        Параметры эксперимента:
        Нет параметров
        
        Returns:
            Список путей к сгенерированным конфигам
        
        Пример использования:
            config_paths = []
            
            # Обновляем один параметр
            config = self.update_parameter("param_name", value)
            path = self.save_config(config, f"param_name_{value}")
            config_paths.append(path)
            
            # Или в цикле для нескольких значений
            for value in [1.0, 2.0, 3.0]:
                config = self.update_parameter("param_name", value)
                path = self.save_config(config, f"param_name_{value}")
                config_paths.append(path)
                
            return config_paths
        """
        config_path = self.save_config(self.base_config, "base")
        return [config_path]