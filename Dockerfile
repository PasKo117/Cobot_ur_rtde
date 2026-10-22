# 1. Используем стабильный slim-образ Python
FROM python:3.11-slim

# 2. Устанавливаем системные зависимости (GUI, OpenCV, Pygame + компиляторы для ur-rtde)
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-tk \
    libgl1 \
    libegl1 \
    libglib2.0-0 \
    libsdl2-dev \
    libportmidi-dev \
    libsdl2-image-dev \
    libsdl2-mixer-dev \
    libsdl2-ttf-dev \
    && rm -rf /var/lib/apt/lists/*

# 3. Задаем рабочую директорию
WORKDIR /app

# 4. Копируем файл зависимостей камер
COPY requirements-cameras.txt .

# 5. Устанавливаем ВСЕ необходимые пакеты для app.py, robot_control, камер и датчиков
RUN pip install --no-cache-dir -r requirements-cameras.txt \
    customtkinter \
    CTkMessagebox \
    Pillow \
    ur-rtde \
    numpy \
    scipy \
    pygame \
    opencv-python \
    pyserial \
    requests

# 6. Копируем весь проект в контейнер
COPY . .

# Переменная окружения для вывода GUI
ENV DISPLAY=:0

# Точка входа
CMD ["python", "app.py"]
