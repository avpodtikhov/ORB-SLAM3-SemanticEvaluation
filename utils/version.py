from dataclasses import dataclass
import subprocess
import os
from typing import List, Optional, Dict
from datetime import datetime

@dataclass
class SlamVersion:
    """Информация о версии SLAM"""
    tag: str                  # Тег версии (например, v1.0.0)
    description: str          # Описание изменений
    commit_hash: str          # Хеш коммита
    timestamp: str            # Время создания тега
    author: str               # Автор
    changes: bool             # Флаг наличия изменений

class SlamVersionManager:
    """Управление версиями SLAM"""
    def __init__(self, repo_path: str):
        """
        Args:
            repo_path: Путь к репозиторию SLAM
        """
        if not os.path.exists(repo_path):
            raise ValueError(f"Директория SLAM не найдена: {repo_path}")

        self.repo_path = repo_path
        
    def get_all_versions(self) -> List[SlamVersion]:
        """Получает список всех версий из ветки slam"""
        # Получаем все теги из ветки slam
        result = subprocess.run(
            ["git", "tag", "-l", "--sort=-creatordate"],
            cwd=self.repo_path,
            capture_output=True,
            text=True,
            check=True
        )
        
        versions = []
        for tag in result.stdout.strip().split('\n'):
            if tag:
                version_info = self.get_version_info(tag)
                if version_info:
                    versions.append(version_info)
        
        return versions
        
    def get_version_info(self, tag: str) -> Optional[SlamVersion]:
        """Получает информацию о конкретной версии"""
        try:
            # Получаем информацию о теге
            tag_info = subprocess.run(
                ["git", "tag", "-l", "--format=%(contents)::%(taggerdate)::%(taggername)", tag],
                cwd=self.repo_path,
                capture_output=True,
                text=True,
                check=True
            ).stdout.strip()
            
            # Разбираем информацию
            description, date, author = tag_info.split('::', 2)
            
            # Получаем хеш коммита
            commit_hash = self.get_commit_hash(tag)
            
            return SlamVersion(
                tag=tag,
                description=description.strip(),
                commit_hash=commit_hash,
                timestamp=date.strip(),
                author=author.strip(),
                changes=False,
            )
                
        except subprocess.CalledProcessError as e:
            print(f"Ошибка при получении информации о версии {tag}: {e}")
            return None
        except ValueError as e:
            print(f"Ошибка при разборе информации о версии {tag}: {e}")
            return None
    
    def get_commit_hash(self, tag: str) -> str:
        """Получает хеш коммита для тега"""
        return subprocess.run(
            ["git", "rev-list", "-n", "1", tag],
            cwd=self.repo_path,
            capture_output=True,
            text=True,
            check=True
        ).stdout.strip()

    def verify_version(self, tag: str) -> bool:
        """
        Проверяет соответствие текущего кода версии.
        """
        try:
            # Получаем коммит тега используя тот же метод
            tag_commit = self.get_commit_hash(tag)
            
            # Получаем текущий коммит
            current_commit = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=self.repo_path,
                capture_output=True,
                text=True,
                check=True
            ).stdout.strip()
            
            # Проверяем наличие несохраненных изменений
            result = subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=self.repo_path,
                capture_output=True,
                text=True
            )
            has_changes = bool(result.stdout.strip())
            
            # Выводим отладочную информацию
            print("\nПроверка версии:")
            print(f"Коммит тега:     {tag_commit}")
            print(f"Текущий коммит:  {current_commit}")
            print(f"Коммиты совпадают: {tag_commit == current_commit}")
            print(f"Есть изменения:    {has_changes}")
            
            return (
                tag_commit == current_commit and
                not has_changes
            )
    
        except subprocess.CalledProcessError as e:
            print(f"Ошибка при проверке версии: {e}")
            return False

    def get_current_state(self) -> Dict:
        """Получает информацию о текущем состоянии"""
        try:
            # Получаем текущий коммит
            current_commit = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=self.repo_path,
                capture_output=True,
                text=True,
                check=True
            ).stdout.strip()
            
            # Проверяем наличие изменений
            result = subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=self.repo_path,
                capture_output=True,
                text=True
            )
            has_changes = bool(result.stdout.strip())
            
            return {
                "commit": current_commit,
                "has_changes": has_changes,
                "timestamp": datetime.now().isoformat(),
            }
            
        except subprocess.CalledProcessError:
            return {}