import os
import re
from typing import List, Dict

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

TEMPLATE_GENERATOR = '''from experiments import register_experiment
from experiments.base import BaseExperiment, ExperimentMetadata

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
            parameters={parameters},
            datasets={datasets}
        )
    
    def generate_configs(self) -> None:
        """
        Генерация YAML конфигураций для эксперимента.
        
        Параметры эксперимента:
        {parameter_docs}
        
        Пример использования:
            config = self.base_config.copy()
            config.update({{
                "ORBextractor": {{
                    "param1": value1,
                    "param2": value2
                }}
            }})
            self.save_config(config, f"config_name")
        """
        raise NotImplementedError(
            "Необходимо реализовать метод generate_configs для эксперимента {experiment_name}!"
            "\\nПараметры для перебора: {parameters}"
        )
'''

TEMPLATE_README = '''# {experiment_name}

## Description
{description}

## Parameters
{parameter_docs}

## Datasets
{datasets_docs}

## Notes
- Add any additional notes or observations here
'''

def sanitize_identifier(name: str) -> str:
    """Преобразует строку в допустимый Python-идентификатор"""
    # Заменяем пробелы на подчеркивания и удаляем недопустимые символы
    identifier = re.sub(r'[^a-zA-Z0-9_]', '', name.replace(' ', '_'))
    # Убеждаемся, что идентификатор начинается с буквы
    if identifier and not identifier[0].isalpha():
        identifier = 'exp_' + identifier
    return identifier

def get_user_input(prompt: str, default: str = None, required: bool = False) -> str:
    """Получает ввод от пользователя с опциональным значением по умолчанию"""
    while True:
        if default:
            prompt = f"{prompt} [{default}]: "
        else:
            prompt = f"{prompt}: "
        
        response = input(prompt).strip()
        
        if not response and default:
            return default
        elif not response and required:
            print("Это поле обязательно для заполнения. Пожалуйста, введите значение.")
            continue
        return response

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
                # Если выбран 0, добавляем все датасеты
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

def generate_parameter_iterations(parameters: dict) -> str:
    """Генерирует код для итерации по параметрам"""
    if not parameters:
        return "pass"
    
    lines = []
    for param_name, param_values in parameters.items():
        lines.append(f"{param_name}_values = {param_values}")
    
    param_names = list(parameters.keys())
    indent = "        "
    
    iterations = []
    for i, param in enumerate(param_names):
        iterations.append(f"{indent}{'    ' * i}for {param} in {param}_values:")
    
    return "\n".join(lines + iterations)

def create_experiment():
    """Создает новый эксперимент на основе пользовательского ввода"""
    print("\n=== Создание нового эксперимента ===")
    
    # Получаем основную информацию
    while True:
        exp_name = get_user_input("Название эксперимента (на английском)", required=True)
        exp_id = sanitize_identifier(exp_name.lower())
        exp_dir = os.path.join('experiments', exp_id)
        
        if os.path.exists(exp_dir):
            print(f"\nОшибка: Эксперимент с названием '{exp_name}' уже существует.")
            print("Пожалуйста, выберите другое название.\n")
            continue
        break
        
    exp_class = sanitize_identifier(exp_name) + 'Experiment'
    
    description = get_user_input("Описание эксперимента", required=True)
    
    # Получаем параметры
    print("\n=== Определение параметров эксперимента ===")
    print("Введите параметры в формате:")
    print("  Имя параметра: список значений (например: [1.0, 2.0, 3.0])")
    print("Оставьте поле пустым для завершения\n")
    
    parameters = {}
    while True:
        param_name = get_user_input("Имя параметра")
        if not param_name:
            break
        param_values = get_user_input(f"Значения для {param_name}", "[1.0, 2.0, 3.0]")
        parameters[param_name] = param_values
        print(f"Добавлен параметр: {param_name} = {param_values}\n")
    
    # Получаем список датасетов
    datasets = get_datasets_input()
    
    # Создаем директорию эксперимента
    os.makedirs(exp_dir, exist_ok=True)
    
    # Создаем __init__.py
    with open(os.path.join(exp_dir, '__init__.py'), 'w') as f:
        f.write('')
    
    # Создаем generator.py с обновленным содержимым
    parameter_docs = "\n        ".join([
        f"{param}: {values}" for param, values in parameters.items()
    ]) or "Нет параметров"
    
    generator_content = TEMPLATE_GENERATOR.format(
        experiment_id=exp_id,
        experiment_class=exp_class,
        experiment_name=exp_name,
        description=description,
        parameters=parameters,
        parameter_docs=parameter_docs,
        datasets=datasets
    )
    
    with open(os.path.join(exp_dir, 'generator.py'), 'w') as f:
        f.write(generator_content)
    
    # Создаем README.md
    parameter_docs = "\n".join([f"- `{param}`: {values}" 
                              for param, values in parameters.items()])
    datasets_docs = "\n".join([f"- {dataset}" for dataset in datasets])
    
    readme_content = TEMPLATE_README.format(
        experiment_name=exp_name,
        description=description,
        parameter_docs=parameter_docs or "No parameters defined",
        datasets_docs=datasets_docs or "No datasets specified"
    )
    
    with open(os.path.join(exp_dir, 'README.md'), 'w') as f:
        f.write(readme_content)
    
    print('\n=== Эксперимент успешно создан ===')
    print(f"Директория: {exp_dir}")
    print("\nСозданные файлы:")
    print(f"  - {os.path.join(exp_dir, '__init__.py')}")
    print(f"  - {os.path.join(exp_dir, 'generator.py')}")
    print(f"  - {os.path.join(exp_dir, 'README.md')}")
    
    print("\nСледующие шаги:")
    print("1. Реализуйте метод generate_configs в generator.py")
    print("2. Метод должен генерировать YAML конфигурации для следующих параметров:")
    if parameters:
        for param, values in parameters.items():
            print(f"   - {param}: {values}")
    else:
        print("   Параметры не определены")

if __name__ == "__main__":
    create_experiment()