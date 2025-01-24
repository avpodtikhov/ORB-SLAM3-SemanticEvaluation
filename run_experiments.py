import os
import json
import datetime
from typing import List, Dict
from dotenv import load_dotenv
from experiments import registry
from utils.slam import run_experiment

load_dotenv()

required_env_vars = {
    'ORB_SLAM_PATH': 'путь к исполняемому файлу ORB-SLAM',
    'ORB_SLAM_VOCABULARY': 'путь к файлу словаря ORB-SLAM'
}

missing_vars = [
    var for var, description in required_env_vars.items() 
    if not os.getenv(var)
]

if missing_vars:
    print("Ошибка: не найдены необходимые переменные окружения!")
    print("Создайте файл .env в корневой директории проекта со следующими переменными:")
    for var in missing_vars:
        print(f"  {var}={required_env_vars[var]}")
    exit(1)

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(PROJECT_ROOT, "data")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "results")

ORB_PATH = os.getenv('ORB_SLAM_PATH')
VOC_PATH = os.getenv('ORB_SLAM_VOCABULARY')

def get_datasets() -> List[str]:
    """Получение списка путей к наборам данных"""
    return sorted([
        os.path.join(DATA_DIR, d) for d in os.listdir(DATA_DIR)
        if os.path.isdir(os.path.join(DATA_DIR, d))
    ])

def select_experiments() -> Dict[str, any]:
    """Интерактивный выбор экспериментов для запуска"""
    experiments = registry.get_all_experiments()
    
    if not experiments:
        print("Эксперименты не найдены!")
        return {}
    
    print("\nДоступные эксперименты:")
    for i, (name, exp_class) in enumerate(experiments.items(), 1):
        metadata = exp_class(os.path.join(PROJECT_ROOT, "experiments", name)).get_metadata()
        print(f"{i}. {metadata.name}")
        print(f"   Описание: {metadata.description.strip()}")
        print(f"   Датасеты: {', '.join(metadata.datasets)}")
        print(f"   Параметры: {metadata.parameters}\n")
    
    selected = {}
    while True:
        try:
            response = input("Введите номера экспериментов через пробел (Enter для запуска всех): ").strip()
            if not response:
                return experiments
            
            numbers = [int(x) for x in response.split()]
            if not all(1 <= n <= len(experiments) for n in numbers):
                print("Некорректные номера экспериментов!")
                continue
                
            selected = {
                name: exp_class
                for i, (name, exp_class) in enumerate(experiments.items(), 1)
                if i in numbers
            }
            break
            
        except ValueError:
            print("Пожалуйста, введите числа через пробел")
    
    return selected

def get_version_info() -> dict:
    """Получает информацию о версии от пользователя"""
    print("\nИнформация о версии ORB-SLAM:")
    
    # Спрашиваем об изменениях
    while True:
        changed = input("Были ли изменения в ORB-SLAM? (y/n): ").lower().strip()
        if changed in ('y', 'n'):
            break
        print("Пожалуйста, введите 'y' или 'n'")
    
    if changed == 'n':
        return {
            "version_tag": "base",
            "description": "Базовая версия ORB-SLAM без изменений",
            "changes": False,
            "timestamp": datetime.datetime.now().isoformat()
        }
    
    # Получаем информацию об изменениях
    while True:
        version_tag = input("Введите тег версии (например, 'fast_matching' или 'opt_ba'): ").strip()
        if version_tag and version_tag.replace('_', '').isalnum():
            break
        print("Тег должен содержать только буквы, цифры и знаки подчеркивания")
    
    description = input("Краткое описание изменений: ").strip()
    while not description:
        print("Описание не может быть пустым")
        description = input("Краткое описание изменений: ").strip()
    
    return {
        "version_tag": version_tag,
        "description": description,
        "changes": True,
        "timestamp": datetime.datetime.now().isoformat()
    }

def create_version_dir(exp_name: str, version_info: dict) -> str:
    """
    Создает новую версию директории для результатов эксперимента
    
    Args:
        exp_name: Имя эксперимента
        version_info: Информация о версии ORB-SLAM
    
    Returns:
        str: Путь к созданной директории
    """
    base_dir = os.path.join(RESULTS_DIR, exp_name)
    
    # Создаем директорию версии
    version_tag = version_info['version_tag']
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    version_dir = os.path.join(base_dir, f"{version_tag}_{timestamp}")
    
    os.makedirs(version_dir, exist_ok=True)
    
    # Создаем символическую ссылку latest
    latest_link = os.path.join(base_dir, "latest")
    if os.path.exists(latest_link):
        os.remove(latest_link)
    os.symlink(version_dir, latest_link)
    
    return version_dir

def run_experiments():
    """Запуск выбранных экспериментов"""
    # Проверяем существование файлов
    if not os.path.exists(ORB_PATH):
        print(f"Ошибка: не найден исполняемый файл ORB-SLAM по пути: {ORB_PATH}")
        return
    
    if not os.path.exists(VOC_PATH):
        print(f"Ошибка: не найден файл словаря ORB-SLAM по пути: {VOC_PATH}")
        return
    
    # Получаем информацию о версии
    version_info = get_version_info()
    
    experiments = select_experiments()
    if not experiments:
        return
        
    print(f"\nЗапуск {len(experiments)} экспериментов: {list(experiments.keys())}")
    print(f"Используется ORB-SLAM: {ORB_PATH}")
    print(f"Версия: {version_info['version_tag']}")
    if version_info['changes']:
        print(f"Изменения: {version_info['description']}")
    
    datasets = get_datasets()
    
    for exp_name, exp_class in experiments.items():
        exp_path = os.path.join(PROJECT_ROOT, "experiments", exp_name)
        experiment = exp_class(exp_path)
        metadata = experiment.get_metadata()
        
        print(f"\nЗапуск эксперимента: {metadata.name}")
        print(f"Описание: {metadata.description.strip()}")
        
        # Генерируем конфигурации
        print("Генерация конфигураций...")
        experiment.generate_configs()
        
        # Получаем список конфигураций
        configs_dir = os.path.join(exp_path, "configs")
        configs = sorted([
            os.path.join(configs_dir, f) 
            for f in os.listdir(configs_dir) 
            if f.endswith('.yaml')
        ])
        
        # Создаем новую версию директории для результатов
        output_path = create_version_dir(exp_name, version_info)
        
        # Сохраняем метаданные и информацию о версии
        version_info.update({
            "metadata": metadata.__dict__,
            "configs_count": len(configs),
            "datasets": metadata.datasets
        })
        
        with open(os.path.join(output_path, "version_info.json"), 'w') as f:
            json.dump(version_info, f, indent=2, ensure_ascii=False)
        
        print(f"Конфигураций для запуска: {len(configs)}")
        print(f"Датасетов для тестирования: {len(metadata.datasets)}")
        
        # Запускаем эксперимент
        run_experiment(
            orb_path=ORB_PATH,
            voc_path=VOC_PATH,
            experiment_configs=configs,
            datasets=datasets,
            base_output_path=output_path
        )
        
        print(f"Эксперимент {metadata.name} завершен!")
        print(f"Результаты сохранены в: {output_path}")

if __name__ == "__main__":
    run_experiments()