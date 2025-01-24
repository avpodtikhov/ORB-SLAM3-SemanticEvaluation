from typing import Dict, Type
from .base import BaseExperiment

class ExperimentRegistry:
    def __init__(self):
        self._experiments: Dict[str, Type[BaseExperiment]] = {}

    def register(self, name: str):
        """Декоратор для регистрации экспериментов"""
        def decorator(cls):
            if not issubclass(cls, BaseExperiment):
                raise TypeError(f"Experiment {name} must inherit from BaseExperiment")
            self._experiments[name] = cls
            return cls
        return decorator

    def get_experiment(self, name: str) -> Type[BaseExperiment]:
        """Получение класса эксперимента по имени"""
        if name not in self._experiments:
            raise KeyError(f"Experiment {name} not found")
        return self._experiments[name]

    def get_all_experiments(self) -> Dict[str, Type[BaseExperiment]]:
        """Получение всех зарегистрированных экспериментов"""
        return self._experiments.copy()