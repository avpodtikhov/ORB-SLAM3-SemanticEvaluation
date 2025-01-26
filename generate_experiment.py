import os
import re
import yaml
from typing import List, Dict, Optional
from utils.version import SlamVersionManager
from utils.config import ConfigManager
import string
from utils.functions import get_user_input

# Словарь доступных датасетов
AVAILABLE_DATASETS = {
    2: "Carla/2",
    3: "Carla/3", 
    4: "Carla/4",
    5: "Carla/5",
    6: "Carla/6",
    7: "Carla/7",
    8: "Carla/8",
}

TEMPLATE_EXPERIMENT = '''from experiments import register_experiment
from experiments.base import BaseExperiment, ExperimentMetadata
from typing import List

@register_experiment("{experiment_id}")
class {experiment_class}(BaseExperiment):
    """
    {description}
    """
    
    def get_metadata(self) -> ExperimentMetadata:
        return ExperimentMetadata(
            name="{experiment_name}",
            description=\"\"\"
            {description}
            \"\"\",
            default_slam_tag="{slam_tag}",
            datasets={datasets},
            parameters={parameters}
        )
    
    def generate_configs(self) -> List[str]:
        """
        Генерация YAML конфигураций для эксперимента.
        
        Параметры эксперимента:
        {parameter_docs}
        
        Returns:
            Список путей к сгенерированным конфигам
        
        Пример использования:
            config_paths = []
            
            # Обновляем один параметр
            config = self.update_parameter("param_name", value)
            path = self.save_config(config, f"param_name_{{value}}")
            config_paths.append(path)
            
            # Или в цикле для нескольких значений
            for value in [1.0, 2.0, 3.0]:
                config = self.update_parameter("param_name", value)
                path = self.save_config(config, f"param_name_{{value}}")
                config_paths.append(path)
                
            return config_paths
        """
        raise NotImplementedError(
            "Необходимо реализовать метод generate_configs для эксперимента {experiment_name}!"
            "\\nПараметры для перебора: {parameters}"
        )
'''

TEMPLATE_README = '''# {experiment_name}

## Description
{description}

## SLAM Version
{slam_tag}

## Parameters
{parameter_docs}

## Datasets
{datasets_docs}

## Notes
- Add any additional notes or observations here
'''

def is_english_letter(char: str) -> bool:
    """
    Проверяет, является ли символ английской буквой
    
    Args:
        char: Проверяемый символ
        
    Returns:
        True если символ - английская буква, False иначе
    """
    return char in string.ascii_letters

def is_pascal_case(text: str) -> bool:
    """
    Проверяет, соответствует ли текст PascalCase и содержит только английские буквы
    
    Args:
        text: Проверяемый текст
        
    Returns:
        True если текст в PascalCase и на английском, False иначе
    """
    # Проверяем что строка не пустая и начинается с большой буквы
    if not text or not text[0].isupper():
        return False
    
    # Проверяем что строка содержит только английские буквы и цифры
    if not all(c.isdigit() or is_english_letter(c) for c in text):
        return False
    
    # Проверяем что нет последовательных заглавных букв (кроме аббревиатур)
    for i in range(1, len(text)-1):
        if (text[i].isupper() and text[i+1].isupper() and 
            not (text[i-1].isupper() and text[i+1].isupper())):
            return False
    
    return True

def pascal_to_snake(text: str) -> str:
    """
    Преобразует PascalCase в snake_case
    
    Args:
        text: Текст в PascalCase
        
    Returns:
        Текст в snake_case
    """
    result = text[0].lower()
    for char in text[1:]:
        if char.isupper():
            result += '_' + char.lower()
        else:
            result += char
    return result


def get_experiment_name() -> tuple[str, str]:
    """
    Получает название эксперимента от пользователя
    
    Returns:
        Tuple[experiment_class, experiment_name]
    """
    while True:
        name = get_user_input(
            "Название эксперимента (в PascalCase, на английском, например: FeatureAnalysis)", 
            required=True
        )
        
        if not is_pascal_case(name):
            print("\nОшибка: Название должно быть в PascalCase и содержать только английские буквы!")
            print("Примеры правильных названий:")
            print("  - FeatureAnalysis")
            print("  - OrbParameterTuning")
            print("  - SemanticMapping")
            print("  - SLAM2D")  # пример с аббревиатурой
            continue
            
        experiment_class = name
        experiment_name = pascal_to_snake(name)
        
        print(f"\nКласс эксперимента: {experiment_class}")
        print(f"Имя эксперимента: {experiment_name}")
        
        if get_user_input("\nПодтвердить? (y/n)", "y").lower() == "y":
            return experiment_class, experiment_name

def get_available_slam_versions() -> Dict[str, str]:
    """
    Получает список доступных версий SLAM
    Returns:
        Dict[tag, description]
    """
    version_manager = SlamVersionManager("../slam")
    versions = version_manager.get_all_versions()
    return {v.tag: v.description for v in versions}

def get_base_config(slam_tag: str) -> Optional[Dict]:
    """
    Получает базовый конфиг для версии SLAM из configs
    """
    config_path = os.path.join("configs", f"{slam_tag}.yaml")
    try:
        config_manager = ConfigManager(config_path)
    except FileNotFoundError:
        return None
    return config_manager.load_base_config()

def get_slam_tag() -> str:
    """Получает тег версии SLAM от пользователя"""
    versions = get_available_slam_versions()
    
    print("\n=== Выбор версии SLAM ===")
    print("Доступные версии в git:")
    for i, (tag, desc) in enumerate(versions.items(), 1):
        print(f"  {i}. {tag} - {desc}")
    
    while True:
        try:
            choice = int(get_user_input("\nВыберите номер версии", "1"))
            if 1 <= choice <= len(versions):
                tag = list(versions.keys())[choice-1]
                
                # Проверяем наличие базового конфига
                if not get_base_config(tag):
                    print(f"\nОшибка: Не найден базовый конфиг для версии {tag}")
                    print(f"Пожалуйста, добавьте файл {tag}.yaml в configs/")
                    exit(1)
                    
                return tag
                
            print(f"Пожалуйста, выберите число от 1 до {len(versions)}")
        except ValueError:
            print("Пожалуйста, введите число")

def get_experiment_parameters(slam_tag: str) -> Dict[str, str]:
    """Получает параметры для эксперимента от пользователя"""
    config = get_base_config(slam_tag)
    parameters = {}
    
    # Показываем доступные параметры
    print(f"\n=== Доступные параметры версии {slam_tag} ===")
    param_list = [(key, str(value)) for key, value in config.items() 
                  if isinstance(value, (int, float, str))]
    
    for i, (name, value) in enumerate(param_list, 1):
        print(f"{i}. {name}")
        print(f"   Текущее значение: {value}\n")
    
    # Выбор параметров
    print("\nВведите номера параметров через пробел или запятую")
    # print("0 - выбрать все параметры")
    
    while True:
        try:
            response = input("\nВыберите параметры: ").strip()
            if not response:
                break
                
            numbers = [int(n) for n in response.replace(',', ' ').split()]
            
            if 0 in numbers:
                selected_params = param_list
            else:
                selected_params = [param_list[num-1] for num in numbers 
                                 if 1 <= num <= len(param_list)]
            
            # Получаем значения для выбранных параметров
            for name, current_value in selected_params:
                values = get_user_input(
                    f"Значения для {name} (текущее: {current_value})",
                    f"{current_value}"
                )
                parameters[name] = values
            
            break
                
        except ValueError:
            print("Ошибка: Введите номера через пробел или запятую")
            
    return parameters

def get_datasets_input() -> List[str]:
    """Получает список датасетов от пользователя по номерам"""
    print("\n=== Выбор датасетов ===")
    print("Доступные датасеты (введите номера через пробел или запятую):")
    print("  0. Все датасеты")
    for num, dataset in AVAILABLE_DATASETS.items():
        print(f"  {num}. {dataset}")
    
    selected_datasets = []
    while not selected_datasets:
        try:
            response = input("\nВыберите датасеты: ").strip()
            numbers = [int(n) for n in re.split(r'[,\s]+', response)]
            
            if 0 in numbers:
                selected_datasets = list(AVAILABLE_DATASETS.values())
            else:
                for num in numbers:
                    if num in AVAILABLE_DATASETS:
                        dataset = AVAILABLE_DATASETS[num]
                        if dataset not in selected_datasets:
                            selected_datasets.append(dataset)
                        else:
                            print(f"Предупреждение: Датасет {dataset} уже выбран")
                    else:
                        print(f"Предупреждение: Неверный номер датасета: {num}")
            
            if not selected_datasets:
                print("Необходимо выбрать хотя бы один датасет!")
            else:
                print("\nВыбранные датасеты:")
                for dataset in selected_datasets:
                    print(f"  - {dataset}")
                
                if get_user_input("\nПодтвердить выбор? (y/n)", "y").lower() == "y":
                    break
                selected_datasets = []
                
        except ValueError:
            print("Ошибка: Введите номера датасетов через пробел или запятую")
    
    return selected_datasets

def create_experiment():
    """Создает новый эксперимент на основе пользовательского ввода"""
    print("\n=== Создание нового эксперимента ===")
    
    # Получаем основную информацию
    while True:
        # Получаем название эксперимента
        exp_class, exp_name = get_experiment_name()
        exp_id = exp_name  # используем snake_case версию как id
        
        # Проверяем существование директории
        exp_dir = os.path.join('experiments', exp_id)
        if os.path.exists(exp_dir):
            print(f"\nОшибка: Эксперимент с именем '{exp_name}' уже существует.")
            print("Пожалуйста, выберите другое название.\n")
            return
        break
    
    exp_class = exp_class + 'Experiment'
    description = get_user_input("Описание эксперимента", required=True)
    
    # Получаем версию SLAM и параметры
    slam_tag = get_slam_tag()
    parameters = get_experiment_parameters(slam_tag)
    
    # Получаем список датасетов
    datasets = get_datasets_input()
    
    # Создаем директории эксперимента
    exp_dir = os.path.join('experiments', exp_id)
    configs_dir = os.path.join(exp_dir, 'configs')
    os.makedirs(exp_dir, exist_ok=True)
    os.makedirs(configs_dir, exist_ok=True)
    
    # Создаем __init__.py
    with open(os.path.join(exp_dir, '__init__.py'), 'w') as f:
        f.write('')
    
    # Создаем experiment.py
    parameter_docs = "\n        ".join([
        f"{param}: {values}" for param, values in parameters.items()
    ]) or "Нет параметров"
    
    experiment_content = TEMPLATE_EXPERIMENT.format(
        experiment_id=exp_id,
        experiment_class=exp_class,
        experiment_name=exp_name,
        description=description,
        parameters=parameters,
        parameter_docs=parameter_docs,
        datasets=datasets,
        slam_tag=slam_tag
    )
    
    with open(os.path.join(exp_dir, 'experiment.py'), 'w') as f:
        f.write(experiment_content)
    
    # Создаем README.md
    parameter_docs = "\n".join([f"- `{param}`: {values}" 
                              for param, values in parameters.items()])
    datasets_docs = "\n".join([f"- {dataset}" for dataset in datasets])
    
    readme_content = TEMPLATE_README.format(
        experiment_name=exp_class,
        description=description,
        parameter_docs=parameter_docs or "No parameters defined",
        datasets_docs=datasets_docs or "No datasets specified",
        slam_tag=slam_tag
    )
    
    with open(os.path.join(exp_dir, 'README.md'), 'w') as f:
        f.write(readme_content)
    
    print('\n=== Эксперимент успешно создан ===')
    print(f"Директория: {exp_dir}")
    print("\nСозданные файлы:")
    print(f"  - {os.path.join(exp_dir, '__init__.py')}")
    print(f"  - {os.path.join(exp_dir, 'experiment.py')}")
    print(f"  - {os.path.join(exp_dir, 'README.md')}")
    print(f"  - {os.path.join(exp_dir, 'configs')} (директория)")
    
    print("\nСледующие шаги:")
    print("1. Реализуйте метод generate_configs в experiment.py")
    print("2. Метод должен генерировать YAML конфигурации для следующих параметров:")
    if parameters:
        for param, values in parameters.items():
            print(f"   - {param}: {values}")
    else:
        print("   Параметры не определены")

if __name__ == "__main__":
    create_experiment()