from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, List
import os
from utils.config import ConfigManager

@dataclass
class ExperimentMetadata:
    """Метаданные эксперимента"""
    name: str
    description: str
    default_slam_tag: str
    datasets: List[str]
    parameters: Dict[str, Any]

class BaseExperiment(ABC):
    """Базовый класс для экспериментов"""
    
    def __init__(self, base_path: str):
        """
        Args:
            base_path: Путь к директории эксперимента
        """
        self.base_path = base_path
        self.configs_path = os.path.join(base_path, "configs")
        os.makedirs(self.configs_path, exist_ok=True)
        
        # Получаем метаданные для определения версии SLAM
        metadata = self.get_metadata()
        
        # Загружаем базовый конфиг для нужной версии
        base_config_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), 
            "configs",
            f"{metadata.default_slam_tag}.yaml"
        )
        
        self.config_manager = ConfigManager(base_config_path)
        self.base_config = self.config_manager.load_base_config()
    
    @abstractmethod
    def generate_configs(self) -> None:
        """Генерация YAML конфигураций для эксперимента"""
        pass
    
    @abstractmethod
    def get_metadata(self) -> ExperimentMetadata:
        """Получение метаданных эксперимента"""
        pass

    @abstractmethod
    def generate_configs(self) -> List[str]:
        """
        Генерация YAML конфигураций для эксперимента
        
        Returns:
            Список путей к сгенерированным конфигам
        """
        pass

    def save_config(self, config: Dict[str, Any], name: str) -> str:
        """
        Сохранение конфигурации в YAML файл
        
        Args:
            config: Конфигурация для сохранения
            name: Имя файла (без расширения)
            
        Returns:
            Путь к сохраненному конфигу
        """
        path = os.path.join(self.configs_path, f"{name}.yaml")
        self.config_manager.save_config(config, path)
        return path