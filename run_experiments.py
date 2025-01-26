import os
from pathlib import Path
from typing import List, Optional, Dict
from dotenv import load_dotenv
from experiments import registry
from utils.slam import SLAM
from utils.version import SlamVersionManager
from utils.config import ConfigManager
from utils.functions import get_user_input
import subprocess

# Загружаем переменные окружения
load_dotenv()

def select_experiment() -> str:
    """
    Позволяет пользователю выбрать эксперимент из списка доступных
    
    Returns:
        ID выбранного эксперимента
    """
    # Получаем все эксперименты
    experiments = registry.get_all_experiments()
    
    if not experiments:
        raise RuntimeError("Нет доступных экспериментов")
    
    # Создаем упорядоченный список экспериментов
    experiment_list = sorted(experiments.items())

    print("\nДоступные эксперименты:")
    for idx, (exp_id, exp) in enumerate(experiment_list, 1):
        try:
            # Получаем метаданные через метод экземпляра
            metadata = exp.get_metadata()
            print(f"\n{idx}. {exp_id}:")
            print(f"  Описание: {' '.join(metadata.description.split())}")
            print(f"  Версия SLAM: {metadata.default_slam_tag}")
            print(f"  Датасеты: {', '.join(metadata.datasets)}")
        except Exception as e:
            print(f"\n{idx}. {exp_id}: Ошибка получения метаданных - {str(e)}")
    
    while True:
        choice = input("\nВыберите эксперимент (номер или ID): ").strip()
        
        # Пробуем интерпретировать как номер
        try:
            idx = int(choice)
            if 1 <= idx <= len(experiment_list):
                return experiment_list[idx-1][0]
            print(f"Номер должен быть от 1 до {len(experiment_list)}")
            continue
        except ValueError:
            # Если не номер, проверяем как ID
            if choice in experiments:
                return choice
            print("Некорректный номер или ID эксперимента")

def verify_slam_version(experiment_id: str) -> None:
    """
    Проверяет соответствие версии SLAM для эксперимента
    
    Args:
        experiment_id: ID эксперимента
        
    Raises:
        RuntimeError: Если версия не соответствует или есть несохраненные изменения
    """
    # Получаем путь к репозиторию SLAM
    slam_repo = os.getenv('ORB_SLAM_REPO')
    if not slam_repo:
        raise ValueError(
            "Не указан путь к репозиторию SLAM. "
            "Укажите в .env (ORB_SLAM_REPO)."
        )
    
    # Получаем требуемую версию из метаданных эксперимента
    experiment = registry.get_experiment(experiment_id)
    metadata = experiment.get_metadata()
    required_version = metadata.default_slam_tag
    
    # Проверяем версию
    version_manager = SlamVersionManager(slam_repo)
    if not version_manager.verify_version(required_version):
        state = version_manager.get_current_state()
        raise RuntimeError(
            f"Версия SLAM не соответствует требуемой ({required_version})\n"
            f"Текущий коммит: {state.get('commit', 'неизвестно')}\n"
            f"Есть изменения: {'да' if state.get('has_changes') else 'нет'}"
        )

def build_slam(repo_path: str) -> None:
    """
    Собирает ORB-SLAM
    
    Args:
        repo_path: Путь к репозиторию SLAM
        
    Raises:
        RuntimeError: Если сборка завершилась с ошибкой
    """
    try:
        print("\nНачинаем сборку ORB-SLAM...")
        build_script = os.path.join(repo_path, 'build.sh')
        
        # Проверяем наличие скрипта сборки
        if not os.path.exists(build_script):
            raise RuntimeError(f"Скрипт сборки не найден: {build_script}")
        
        # Делаем скрипт исполняемым
        os.chmod(build_script, 0o755)
        
        # Запускаем сборку
        process = subprocess.run(
            ['./build.sh'],
            cwd=repo_path,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )
        
        if process.returncode != 0:
            raise RuntimeError(
                f"Ошибка при сборке (код {process.returncode}):\n{process.stdout}"
            )
            
        print("Сборка успешно завершена")
        
    except subprocess.SubprocessError as e:
        raise RuntimeError(f"Ошибка при выполнении сборки: {str(e)}")

def verify_and_prepare_slam(experiment_id: str) -> None:
    """
    Проверяет версию SLAM и при необходимости выполняет сборку
    
    Args:
        experiment_id: ID эксперимента
    """
    # Получаем пути из .env
    repo_path = os.getenv('ORB_SLAM_REPO')
    orb_path = os.path.join(repo_path, os.getenv('ORB_SLAM_PATH'))
    
    if not repo_path:
        raise ValueError("Не указан путь к репозиторию SLAM в .env (ORB_SLAM_REPO)")
    
    # Проверяем версию
    version_manager = SlamVersionManager(repo_path)
    experiment = registry.get_experiment(experiment_id)
    metadata = experiment.get_metadata()
    required_version = metadata.default_slam_tag
    
    if not version_manager.verify_version(required_version):
        state = version_manager.get_current_state()
        raise RuntimeError(
            f"Версия SLAM не соответствует требуемой ({required_version})\n"
            f"Текущий коммит: {state.get('commit', 'неизвестно')}\n"
            f"Есть изменения: {'да' if state.get('has_changes') else 'нет'}"
        )
    
    # Проверяем наличие исполняемого файла
    if not os.path.exists(orb_path):
        print(f"\nИсполняемый файл не найден: {orb_path}")
        build_slam(repo_path)


def run_experiment(
    experiment_id: str,
    output_dir: Optional[str] = None,
    max_workers: Optional[int] = None
) -> None:
    """
    Запускает эксперимент
    
    Args:
        experiment_id: ID эксперимента
        output_dir: Директория для результатов
        max_workers: Максимальное количество параллельных процессов
    """
    # Проверяем версию SLAM
    verify_slam_version(experiment_id)
    
    # Получаем пути из .env
    orb_path = os.path.join(os.getenv('ORB_SLAM_REPO'), os.getenv('ORB_SLAM_PATH'))
    voc_path = os.getenv('ORB_SLAM_VOC')
    datasets_dir = os.getenv('DATASETS_DIR')
    
    # Проверяем наличие необходимых путей
    if not orb_path:
        raise ValueError("Не указан путь к ORB-SLAM в .env (ORB_SLAM_PATH)")
    if not voc_path:
        raise ValueError("Не указан путь к словарю в .env (ORB_SLAM_VOC)")
    if not datasets_dir:
        raise ValueError("Не указана директория с датасетами в .env (DATASETS_DIR)")
    
    # Получаем эксперимент и метаданные
    experiment = registry.get_experiment(experiment_id)
    metadata = experiment.get_metadata()
    
    # Генерируем конфиги
    config_paths = experiment.generate_configs()
    print(f"\nСгенерировано {len(config_paths)} конфигураций")
    
    # Формируем пути к датасетам
    datasets = [
        os.path.join(datasets_dir, dataset)
        for dataset in metadata.datasets
    ]
    
    # Проверяем существование датасетов
    for dataset in datasets:
        if not os.path.exists(dataset):
            raise FileNotFoundError(f"Датасет не найден: {dataset}")
    
    # Определяем директорию для результатов
    if output_dir is None:
        output_dir = os.path.join('experiments', experiment_id, 'results')
    
    # Создаем SLAM с путями из .env
    slam = SLAM(orb_path=orb_path, voc_path=voc_path)
    
    # Запускаем эксперимент
    slam.run_experiment(
        configs=config_paths,
        datasets=datasets,
        output_path=output_dir,
        max_workers=max_workers
    )

def main():
    try:
        # Выбор эксперимента
        experiment_id = select_experiment()
        
        # Формируем путь по умолчанию для результатов
        default_output_dir = os.path.join('experiments', experiment_id, 'results')
        
        # Запрос директории для результатов
        output_dir = input(
            f"\nДиректория для результатов "
            f"[{default_output_dir}]: "
        ).strip()
        output_dir = output_dir if output_dir else default_output_dir
        
        # Запрос количества процессов
        default_workers = os.getenv('MAX_WORKERS', '6')
        max_workers = input(
            f"\nКоличество процессов [{default_workers}]: "
        ).strip()
        max_workers = int(max_workers) if max_workers else int(default_workers)
        
        # Запуск эксперимента
        run_experiment(experiment_id, output_dir, max_workers)
        
    except Exception as e:
        print(f"\nОшибка: {str(e)}")
        exit(1)

if __name__ == '__main__':
    main()