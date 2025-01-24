from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, List
import os
import yaml

@dataclass
class ExperimentMetadata:
    name: str
    description: str
    parameters: Dict[str, Any]
    datasets: List[str]

class BaseExperiment(ABC):
    def __init__(self, base_path: str):
        self.base_path = base_path
        self.configs_path = os.path.join(base_path, "configs")
        os.makedirs(self.configs_path, exist_ok=True)
        
        # Загружаем базовый конфиг
        base_config_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), 
            "configs", 
            "base_config.yaml"
        )
        with open(base_config_path) as f:
            self.base_config = yaml.safe_load(f)
    
    @abstractmethod
    def generate_configs(self) -> None:
        """Генерация YAML конфигураций для эксперимента"""
        pass
    
    @abstractmethod
    def get_metadata(self) -> ExperimentMetadata:
        """Получение метаданных эксперимента"""
        pass
    
    def save_config(self, config: Dict[str, Any], name: str) -> None:
        """Сохранение конфигурации в YAML файл"""
        path = os.path.join(self.configs_path, f"{name}.yaml")
        with open(path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)