import os
import importlib
from experiments.base import BaseExperiment, ExperimentMetadata
from experiments.registry import ExperimentRegistry

# Создаем единый реестр экспериментов
registry = ExperimentRegistry()

# Экспортируем декоратор для удобства использования
register_experiment = registry.register

def _import_experiments() -> None:
    """
    Импортирует все модули экспериментов
    
    Структура директории:
    experiments/
        experiment_name/
            __init__.py
            experiment.py
            configs/
            README.md
    """
    current_dir = os.path.dirname(__file__)
    
    for item in os.listdir(current_dir):
        if not os.path.isdir(os.path.join(current_dir, item)):
            continue
            
        if item.startswith('__'):  # Пропускаем специальные директории
            continue
            
        experiment_path = os.path.join(current_dir, item, 'experiment.py')
        if os.path.exists(experiment_path):
            module_name = f"experiments.{item}.experiment"
            try:
                importlib.import_module(module_name)
                # print(f"Загружен эксперимент: {item}")
            except Exception as e:
                print(f"Ошибка загрузки эксперимента {item}: {str(e)}")

# Выполняем импорт при загрузке модуля
_import_experiments()

# Экспортируем для удобства использования
__all__ = [
    'BaseExperiment',
    'ExperimentMetadata',
    'register_experiment',
    'registry'
]