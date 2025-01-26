from concurrent.futures import ProcessPoolExecutor
from functools import partial
import subprocess
import os
import yaml
from typing import List, Optional
from utils.config import ConfigManager
from tqdm.auto import tqdm

class SLAM:
    """Интерфейс для запуска и управления ORB-SLAM"""
    
    def __init__(self, orb_path: Optional[str] = None, voc_path: Optional[str] = None):
        """
        Args:
            orb_path: Путь к исполняемому файлу ORB-SLAM (если не указан, берется из .env)
            voc_path: Путь к словарю (если не указан, берется из .env)
        """
        self.orb_path = orb_path or os.getenv('ORB_SLAM_PATH')
        self.voc_path = voc_path or os.getenv('ORB_SLAM_VOC')
        
        if not self.orb_path or not os.path.exists(self.orb_path):
            raise FileNotFoundError(
                f"Исполняемый файл не найден: {self.orb_path}. "
                "Укажите путь в .env (ORB_SLAM_PATH) или передайте в конструктор."
            )
        
        if not self.voc_path or not os.path.exists(self.voc_path):
            raise FileNotFoundError(
                f"Словарь не найден: {self.voc_path}. "
                "Укажите путь в .env (ORB_SLAM_VOC) или передайте в конструктор."
            )
    
    def run_single(
        self,
        yaml_path: str,
        dataset_path: str,
        output_path: str
    ) -> None:
        """
        Запуск алгоритма на заданном наборе данных
        
        Args:
            yaml_path: Путь к конфигурационному файлу
            dataset_path: Путь к набору данных
            output_path: Путь для сохранения результатов
            
        Raises:
            RuntimeError: Если произошла ошибка при запуске или выполнении
        """
        # Проверяем существование файлов
        if not os.path.exists(yaml_path):
            raise RuntimeError(f"Не найден конфигурационный файл: {yaml_path}")
        if not os.path.exists(dataset_path):
            raise RuntimeError(f"Не найден набор данных: {dataset_path}")
        
        # Создаем директорию для результатов
        config_name = os.path.splitext(os.path.basename(yaml_path))[0]
        dataset_name = os.path.basename(dataset_path)
        result_dir = os.path.join(output_path, dataset_name, config_name)
        os.makedirs(result_dir, exist_ok=True)
        
        # Пути к файлам результатов
        trajectory_path = os.path.join(result_dir, 'trajectory.txt')
        log_path = os.path.join(result_dir, 'log.txt')
        
        try:
            # Запускаем процесс
            with open(log_path, 'w') as log_file:
                process = subprocess.Popen(
                    [
                        self.orb_path,
                        self.voc_path,
                        yaml_path,
                        dataset_path,
                        trajectory_path
                    ],
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
            raise RuntimeError(f"Не удалось найти исполняемый файл: {self.orb_path}")
        except PermissionError:
            raise RuntimeError(f"Нет прав на выполнение файла: {self.orb_path}")
        except subprocess.SubprocessError as e:
            raise RuntimeError(f"Ошибка при выполнении процесса ORB-SLAM: {str(e)}")
    
    def run_experiment(
        self,
        configs: List[str],
        datasets: List[str],
        output_path: str,
        max_workers: Optional[int] = None
    ) -> None:
        """
        Запуск эксперимента на всех конфигурациях и датасетах
        
        Args:
            configs: Список путей к конфигурационным файлам
            datasets: Список путей к наборам данных
            output_path: Путь для сохранения результатов
            max_workers: Максимальное количество параллельных процессов
        """
        total_runs = len(configs) * len(datasets)
        print(f"\nЗапуск {total_runs} процессов "
              f"({len(configs)} конфигураций × {len(datasets)} датасетов)")
        
        max_workers = max_workers or int(os.getenv('MAX_WORKERS', '6'))
        
        try:
            with ProcessPoolExecutor(max_workers=max_workers) as executor:
                futures = []
                
                # Создаем задачи для всех комбинаций
                for dataset_path in datasets:
                    for config_path in configs:
                        process_func = partial(
                            self.run_single,
                            yaml_path=config_path,
                            dataset_path=dataset_path,
                            output_path=output_path
                        )
                        futures.append(executor.submit(process_func))
                
                # Отображаем прогресс
                failed = []
                with tqdm(total=total_runs, desc="Выполнение эксперимента") as pbar:
                    for future in futures:
                        try:
                            future.result()
                        except Exception as e:
                            failed.append(str(e))
                        finally:
                            pbar.update(1)
                
                # Выводим ошибки
                if failed:
                    print("\nПроизошли ошибки при выполнении некоторых процессов:")
                    for error in failed:
                        print(f"- {error}")
        except KeyboardInterrupt:
            print("\nПрерывание выполнения...")
            executor.shutdown(wait=False)
            raise