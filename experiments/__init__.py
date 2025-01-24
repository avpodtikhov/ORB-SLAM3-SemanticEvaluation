import os
import importlib
from .base import BaseExperiment, ExperimentMetadata
from .registry import ExperimentRegistry

# Создаем единый реестр экспериментов
registry = ExperimentRegistry()

# Экспортируем декоратор для удобства использования
register_experiment = registry.register

# Автоматически импортируем все эксперименты
def _import_experiments():
    """Импортирует все модули экспериментов"""
    current_dir = os.path.dirname(__file__)
    
    for item in os.listdir(current_dir):
        if not os.path.isdir(os.path.join(current_dir, item)):
            continue
            
        if item.startswith('__'):  # Пропускаем специальные директории
            continue
            
        generator_path = os.path.join(current_dir, item, 'generator.py')
        if os.path.exists(generator_path):
            module_name = f"experiments.{item}.generator"
            try:
                importlib.import_module(module_name)
                print(f"Loaded experiment: {item}")
            except Exception as e:
                print(f"Failed to load experiment {item}: {str(e)}")

# Выполняем импорт при загрузке модуля
_import_experiments()