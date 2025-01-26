from typing import Dict, Type, List
import os
from experiments.base import BaseExperiment, ExperimentMetadata

class ExperimentRegistry:
    def __init__(self):
        self._experiments: Dict[str, Type[BaseExperiment]] = {}
        self._load_existing_experiments()

    def _load_existing_experiments(self) -> None:
        """Загружает существующие эксперименты из директории experiments"""
        experiments_dir = os.path.dirname(os.path.dirname(__file__))
        
        for item in os.listdir(experiments_dir):
            exp_dir = os.path.join(experiments_dir, item)
            if not os.path.isdir(exp_dir) or item.startswith('__'):
                continue
                
            experiment_path = os.path.join(exp_dir, 'experiment.py')
            if os.path.exists(experiment_path):
                module_name = f"experiments.{item}.experiment"
                try:
                    __import__(module_name)
                except Exception as e:
                    print(f"Ошибка загрузки эксперимента {item}: {str(e)}")

    def register(self, experiment_id: str):
        """
        Декоратор для регистрации экспериментов
        
        Args:
            experiment_id: Уникальный идентификатор эксперимента (в snake_case)
        """
        def decorator(cls):
            if not issubclass(cls, BaseExperiment):
                raise TypeError(f"Эксперимент {experiment_id} должен наследоваться от BaseExperiment")
                
            if not (hasattr(cls, 'get_metadata') and hasattr(cls, 'generate_configs')):
                raise TypeError(
                    f"Эксперимент {experiment_id} должен реализовывать методы "
                    "get_metadata и generate_configs"
                )
                
            self._experiments[experiment_id] = cls
            return cls
        return decorator

    def get_experiment(self, experiment_id: str) -> BaseExperiment:
        """
        Получение экземпляра эксперимента по ID
        
        Args:
            experiment_id: ID эксперимента
            
        Returns:
            Экземпляр эксперимента
            
        Raises:
            KeyError: Если эксперимент не найден
        """
        if experiment_id not in self._experiments:
            raise KeyError(f"Эксперимент {experiment_id} не найден")
            
        exp_class = self._experiments[experiment_id]
        return exp_class(os.path.join('experiments', experiment_id))

    def get_all_experiments(self) -> Dict[str, BaseExperiment]:
        """
        Получение всех зарегистрированных экспериментов
        
        Returns:
            Словарь {experiment_id: experiment_instance}
        """
        return {
            exp_id: exp_class(os.path.join('experiments', exp_id))
            for exp_id, exp_class in self._experiments.items()
        }
        
    def get_experiments_by_version(self, slam_tag: str) -> List[BaseExperiment]:
        """
        Получение всех экспериментов для указанной версии SLAM
        
        Args:
            slam_tag: Тег версии SLAM
            
        Returns:
            Список экземпляров экспериментов
        """
        experiments = []
        for exp_id, exp_class in self._experiments.items():
            try:
                exp = exp_class(os.path.join('experiments', exp_id))
                metadata = exp.get_metadata()
                if metadata.default_slam_tag == slam_tag:
                    experiments.append(exp)
            except Exception as e:
                print(f"Ошибка при получении метаданных эксперимента {exp_id}: {str(e)}")
                continue
        return experiments