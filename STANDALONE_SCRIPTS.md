# Отдельные программы ветки `summertime`

Все команды ниже выполняются **из корня репозитория** в Python 3.12. Для основного окна и управления установите `python -m pip install -r requirements.txt`. Для последовательного порта, сетевой PTZ-камеры, RoboDK и конвертации DOCX дополнительно `python -m pip install -r requirements-extras.txt`; для последней операции нужен исполняемый Pandoc.

Контроллеры: диагност `192.168.8.3`, хирург `192.168.8.4`. Основной путь — `python app.py`. Каждый автономный скрипт, который читает или управляет роботами, берёт блокировку IP через `standalone_rtde.py`: сначала закройте окно и дочерний процесс управления `app.py`, затем запустите **один** автономный скрипт. Блокировка действует между файлами этого репозитория на одном компьютере. Другие ПК и внешние RTDE-клиенты она не блокирует. При закрытии сеанса скорости останавливаются, соединения разрываются. Программы, не требующие робота (камеры, лазер, последовательный порт), этой блокировкой не пользуются.

## Роботы: основной интерфейс и ручные пробы

| Файл | Запуск из корня | Что происходит |
|---|---|---|
| `app.py` | `python app.py` | Основное окно: два джойстика, маршруты, камеры, датчики. Управление у одного процесса. |
| `Joystick_hirurg.py` | `python Joystick_hirurg.py --speed 25` | Консольный запуск того же **двухроботного** процесса и двух джойстиков. Физические кнопки 7/8 — режимы, 11 — точка, 10 — CSV в `routes/`, 12 — воспроизведение. `Ctrl+C` останавливает. |
| `Joystick_diagnost.py` | `python Joystick_diagnost.py --speed 25` | Та же консольная программа для двух джойстиков; старое имя сохранено. Сохранение файлов автоматически в `routes/`. |
| `us_path.py`, `Keyboard/Joystick_8_3.py` | `python us_path.py --speed 25`; `python Keyboard/Joystick_8_3.py --speed 25` | Старые имена запуска этой же консольной программы; нужны оба джойстика (ID 0 — хирург, ID 1 — диагност). |
| `joystick_probe.py`, `delete_it.py` | `python joystick_probe.py`; `python delete_it.py` | Печатают номера нажатых кнопок/их назначения без RTDE и движения. |
| `keyboard_tool.py`, `Keyboard/Tool_keyboard_test.py` | `python keyboard_tool.py --speed 0.01`; `python Keyboard/Tool_keyboard_test.py` | Клавиатура для диагноста **в системе инструмента**; стрелки X/Y, PgUp/PgDown Z, Q/W A/S Z/X вращение; Esc остановка. |
| `speedl_keyboard_83.py`, `Keyboard/speedl_keyboard_test.py` | `python speedl_keyboard_83.py --speed 0.01`; `python Keyboard/speedl_keyboard_test.py` | Та же клавиатура в **системе базы**. Верхний предел `--speed`: 0.05 м/с. |
| `Align_D.py`, `Align_H.py` | `python Align_D.py`; `python Align_H.py` | Показывают целевые позы: диагност +50 мм по Z базы / хирург ориентация `[π/2,0,0]`. `--execute` отправляет `moveL` на соответствующий робот. |
| `get_joints.py`, `telemetry.py` | `python get_joints.py`; `python telemetry.py` | Однократно читают TCP и суставы хирурга / TCP, суставы и усилие диагноста; движения нет. |
| `test.py` | `python test.py` | Показывает прогноз одного шага 5 мм по оси инструмента диагноста. `python test.py --execute` делает шаг и возврат. |
| `us_app.py` | `python us_app.py` | Старое имя GUI; запускает текущее окно `app.py`. `python us_app.py --route --steps 2 --step-mm 1 --pause 0.5 --speed 0.01` проводит пробный путь Y туда/обратно, пишет `data.txt` (CSV). |

`--speed 100` у консольных джойстиков соответствует существующему пределу приложения, а не паспортной скорости UR5e. Консольный режим не меняет скорость во время запуска; для ползунка используйте `app.py`. После сохранения маршрута кнопкой 10 консольный процесс останавливает ручное управление; завершите его `Ctrl+C` и запустите заново. Для чтения файла маршрута используйте кнопку основного окна.

## Калибровка и исследования

| Файл | Запуск | Назначение |
|---|---|---|
| `robot_calibration.py` | `python robot_calibration.py` | Проверяет `calibration_data.json`, движения нет. `--collect --points 5` собирает пять пар общих XYZ-меток во freedrive, рассчитывает `R,t` **диагност → хирург** и сохраняет JSON только если RMS ≤ 5 мм. `--preview` показывает расчётную позу хирурга; `--preview --execute` требует ещё слово `MOVE` в консоли и направляет хирурга к ней. Расчёт не подтверждает отсутствие столкновения. |
| `CAFC.py` | `python CAFC.py` | Читает одно значение сил диагноста. `python CAFC.py --execute --seconds 3` делает ограниченную пробу с обратной связью по Z и сохраняет CSV; коэффициенты исследовательские, медицинским/сертифицированным силовым контролем это не является. |
| `las_exp.py` | `python las_exp.py` | Читает оба лазерных датчика через Raspberry Pi JSON сервер. `--experiment uno|duo|tres --execute --step-mm 1 --speed 0.005` проводит **один** шаг и пишет `experiments/*.csv`; `duo` принимает `--angle-deg 5`. Старый протокол `192.168.8.149:9093` не использует. |

В `Align_*.py`, `test.py`, `las_exp.py` и пробе калибровки движение включается только указанным аргументом. Измерения, значения усилия, пространство двух роботов, углы TCP и калибровка требуют проверки на конкретном стенде. Прерывайте движение штатной остановкой и контролируйте его на пульте.

## Питание и защитная остановка

| Файл | Запуск | Назначение |
|---|---|---|
| `Power_On_D.py`, `Power_On_H.py` | `python Power_On_D.py`, `python Power_On_H.py` | `power on`, ожидание 5 с, `brake release` через Dashboard 29999 для диагноста/хирурга. |
| `Power_Off_D.py`, `Power_Off_H.py` | `python Power_Off_D.py`, `python Power_Off_H.py` | Один `power off` для указанного робота; ожидают ответ Dashboard. |
| `ESTOP_RESET_D.py`, `ESTOP_RESET_H.py` | `python ESTOP_RESET_D.py`, `python ESTOP_RESET_H.py` | Команда `unlock protective stop`. Имя файла историческое: **физический аварийный останов** эта команда не снимает. Сначала выясните причину на пульте. |

## Мониторинг, камеры и внешнее оборудование

| Файл | Запуск | Назначение |
|---|---|---|
| `Tracking_Diagnost.py`, `Tracking_Hirurg.py` | `python Tracking_Diagnost.py --once`; `python Tracking_Hirurg.py --interval 0.1` | RTDE суставы (рад) переводятся в градусы для модели `Diagnost` / `Hirurg` в открытом RoboDK. `--targets` добавляет целевые точки при изменении >10°. Нужен пакет `robodk`. |
| `camDiagn.py`, `camHirurg.py`, `cam.py` | `python camDiagn.py` и т. д. | Отдельные окна камер диагноста / хирурга (старый `cam.py` — хирург). Esc — закрыть, `+`/`-` — зум. Нужен `opencv-python`. |
| `ip_cam.py` | `python ip_cam.py --pan 20` | Одна PTZ-команда для сетевой камеры. Имя и пароль задаются через переменные окружения `COBOT_CAMERA_USER` и `COBOT_CAMERA_PASSWORD`; нет бесконечного качания камеры. Нужен `requests`. |
| `force_measure/Force measure.py` | `python "force_measure/Force measure.py"` | Окно усилия диагноста по RTDE без `urx`. |
| `laser_sens/laser_sens.py` | `python laser_sens/laser_sens.py` | Окно двух лазерных расстояний из `laser_sensor_lib.py`, сервер Pi `192.168.8.37:5000`. |
| `pomogite_1.py` | `python pomogite_1.py --ip 192.168.8.3 --port 6000` | Одно измерение по отдельному бинарному протоколу OD Mini. Требует соответствующий TCP сервер; это **не** JSON сервер Pi. |
| `main.py` | `python main.py --port COM2 --once` | Сырые ответы контроллера по последовательному порту, без интерпретации единиц измерения. Нужен `pyserial`. |
| `scanip.py` | `python scanip.py --ip 192.168.8.3` | Печатает состояние списка TCP портов; не управляет оборудованием. |

## Вспомогательные файлы

`robot_control.py`, `standalone_rtde.py`, `standalone_joystick.py`, `keyboard_rtde.py`, `tracking_rtde.py`, `camera_view.py`, `route_io.py`, `calibration_manager.py`, `laser_sensor_lib.py` — импортируемые модули, самостоятельный запуск не требуется. `joystick_map.json` настраивает номера кнопок `pygame`, `calibration_data.json` хранит вычисленное преобразование. `app.spec` и `us_app.spec` используются командой `pyinstaller app.spec` / `pyinstaller us_app.spec` и содержат JSON и динамически импортируемые модули. `python -m unittest discover -s tests -v` проверяет код без оборудования. `python scripts/convert_docx.py` преобразует `docs/source.docx` в README (нужен Pandoc); ссылка на эту инструкцию сохраняется конвертером.
