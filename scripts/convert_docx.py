# scripts/convert_docx.py
import pypandoc
import os
import sys

# Пути (относительно корня репозитория)
INPUT_FILE = 'docs/source.docx'  # <--- ПРОВЕРЬТЕ ПУТЬ!
OUTPUT_FILE = 'README.md'
MEDIA_DIR = 'media'


def main():
    print("🚀 Начинаю конвертацию...")

    if not os.path.exists(INPUT_FILE):
        print(f"❌ Ошибка: Файл '{INPUT_FILE}' не найден!")
        sys.exit(1)

    try:
        # Создаем папку для изображений
        os.makedirs(MEDIA_DIR, exist_ok=True)

        # Конвертируем и извлекаем медиа-файлы
        output = pypandoc.convert_file(
            INPUT_FILE,
            'markdown',
            extra_args=['--extract-media=' + MEDIA_DIR, '--standalone']
        )

        with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
            f.write(output)

        print("✅ Конвертация успешна!")
        print(f"📄 Создан файл: {OUTPUT_FILE}")
        print(f"🖼️ Изображения сохранены в: {MEDIA_DIR}/")

    except Exception as e:
        print(f"❌ Ошибка при конвертации: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()