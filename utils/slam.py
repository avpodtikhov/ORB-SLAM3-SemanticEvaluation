from concurrent.futures import ProcessPoolExecutor
from functools import partial
import subprocess
import os
import yaml
from typing import List
from tqdm.auto import tqdm

MAX_WORKERS = 6

def run(orb_path: str, voc_path: str, yaml_path: str, dataset_path: str, output_path: str) -> None:
    """
    Запуск алгоритма на заданном наборе данных.
    
    Args:
        orb_path: Путь к исполняемому файлу ORB-SLAM
        voc_path: Путь к словарю
        yaml_path: Путь к конфигурационному файлу
        dataset_path: Путь к набору данных
        output_path: Путь для сохранения результатов
    
    Raises:
        RuntimeError: Если произошла ошибка при запуске или выполнении процесса
    """
    # Проверяем существование всех необходимых файлов
    for path, description in [
        (orb_path, "исполняемый файл ORB-SLAM"),
        (voc_path, "словарь"),
        (yaml_path, "конфигурационный файл"),
        (dataset_path, "набор данных")
    ]:
        if not os.path.exists(path):
            raise RuntimeError(f"Не найден {description}: {path}")
    
    # Создаем директорию для результатов
    config_name = os.path.splitext(os.path.basename(yaml_path))[0]
    dataset_name = os.path.basename(dataset_path)
    result_dir = os.path.join(output_path, dataset_name, config_name)
    os.makedirs(result_dir, exist_ok=True)
    
    # Пути к файлам результатов
    trajectory_path = os.path.join(result_dir, 'trajectory.txt')
    log_path = os.path.join(result_dir, 'log.txt')
    
    try:
        # Сохраняем копию конфигурации
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        with open(os.path.join(result_dir, 'config.yaml'), 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
        
        # Запускаем процесс
        with open(log_path, 'w') as log_file:
            process = subprocess.Popen(
                [orb_path, voc_path, yaml_path, dataset_path, trajectory_path],
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True
            )
            returncode = process.wait()
            
            if returncode != 0:
                raise RuntimeError(
                    f"Процесс ORB-SLAM завершился с ошибкой (код {returncode}). "
                    f"Подробности в файле: {log_path}"
                )
            
    except FileNotFoundError:
        raise RuntimeError(f"Не удалось найти исполняемый файл: {orb_path}")
    except PermissionError:
        raise RuntimeError(f"Нет прав на выполнение файла: {orb_path}")
    except subprocess.SubprocessError as e:
        raise RuntimeError(f"Ошибка при выполнении процесса ORB-SLAM: {str(e)}")

def run_experiment(
    orb_path: str,
    voc_path: str,
    experiment_configs: List[str],
    datasets: List[str],
    base_output_path: str
) -> None:
    """
    Запуск эксперимента на всех конфигурациях и датасетах.
    
    Args:
        orb_path: Путь к исполняемому файлу ORB-SLAM
        voc_path: Путь к словарю
        experiment_configs: Список путей к конфигурационным файлам
        datasets: Список путей к наборам данных
        base_output_path: Базовый путь для сохранения результатов
    """
    total_runs = len(experiment_configs) * len(datasets)
    print(f"\nЗапуск {total_runs} процессов ({len(experiment_configs)} конфигураций × {len(datasets)} датасетов)")
    
    with ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
        futures = []
        
        # Создаем задачи для всех комбинаций конфигураций и датасетов
        for dataset_path in datasets:
            for config_path in experiment_configs:
                process_func = partial(
                    run,
                    orb_path=orb_path,
                    voc_path=voc_path,
                    yaml_path=config_path,
                    dataset_path=dataset_path,
                    output_path=base_output_path
                )
                futures.append(executor.submit(process_func))
        
        # Отображаем прогресс выполнения
        failed = []
        with tqdm(total=total_runs, desc="Выполнение эксперимента") as pbar:
            for future in futures:
                try:
                    future.result()
                except Exception as e:
                    failed.append(str(e))
                finally:
                    pbar.update(1)
        
        # Выводим информацию об ошибках, если они были
        if failed:
            print("\nПроизошли ошибки при выполнении некоторых процессов:")
            for error in failed:
                print(f"- {error}")