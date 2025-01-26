import yaml
import os
from typing import Dict, Any

class ConfigManager:
    """Базовые операции с конфигурациями ORB-SLAM"""
    
    def __init__(self, base_config_path: str):
        """
        Args:
            base_config_path: Путь к базовому конфигу
        """
        if not os.path.exists(base_config_path):
            raise FileNotFoundError(f"Базовый конфиг не найден: {base_config_path}")
        self.base_config_path = base_config_path
    
    def load_base_config(self) -> Dict[str, Any]:
        """Загружает базовый конфиг"""
        with open(self.base_config_path, 'r') as f:
            # Пропускаем первую строку с версией YAML
            f.readline()
            return yaml.safe_load(f)
    
    def save_config(self, config: Dict[str, Any], path: str) -> None:
        """
        Сохраняет конфиг в файл в формате ORB-SLAM
        
        Args:
            config: Конфигурация для сохранения
            path: Путь для сохранения
        """
        os.makedirs(os.path.dirname(path), exist_ok=True)
        
        with open(path, 'w') as f:
            # Записываем заголовок
            f.write("%YAML:1.0\n\n")
            
            # Записываем конфиг
            yaml.dump(config, f, default_flow_style=False)
    
    def merge_configs(self, base_config: Dict[str, Any], update_config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Объединяет два конфига
        
        Args:
            base_config: Базовый конфиг
            update_config: Конфиг с обновлениями
            
        Returns:
            Объединенный конфиг
        """
        result = base_config.copy()
        result.update(update_config)
        return result