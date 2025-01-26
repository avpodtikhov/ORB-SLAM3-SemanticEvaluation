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