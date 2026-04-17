# robot-aix

Монорепозиторий системы управления роботом с разделением на станцию оператора и несколько микроконтроллерных узлов.

Проект объединяет:

- веб-интерфейс станции управления;
- прошивку основного контроллера на базе PRIZM;
- промежуточную прошивку `compute`-контроллера;
- отдельную прошивку шагового контроллера для исполнительного механизма.

## Архитектура

По текущей структуре кода система выглядит так:

```text
Control Station (Vite/React)
        |
        v
PRIZM controller firmware
        |
        v
Compute MCU firmware
        |
        v
Stepper MCU firmware
```

Транспортный слой между узлами по текущим модулям реализован так:

- станция управления работает как внешний интерфейс оператора;
- `prizm_firmware` содержит модуль `station/hc12_backend`, то есть выступает точкой связи со станцией;
- `prizm_firmware` использует `spi_master_backend`, а `compute_mcu_firmware` содержит `spi_slave_backend`;
- `compute_mcu_firmware` использует `i2c_stepper_client`, а `stepper_mcu_firmware` содержит `i2c_slave_server`.

## Состав репозитория

| Каталог | Назначение | Текущий статус |
| --- | --- | --- |
| `ControlStation-main/` | Фронтенд станции управления на `Vite + React + TypeScript` | Есть интерфейс и документация по `request id` |
| `prizm_firmware/` | Основная прошивка контроллера PRIZM | Реализован Stage 1, Stage 2/3 подготовлены частично |
| `compute_mcu_firmware/` | Прошивка промежуточного ATmega328P-контроллера | Реализован Stage 1, есть заготовки для расширения |
| `stepper_mcu_firmware/` | Прошивка шагового контроллера ATmega328P по I2C | Реализовано ядро движения и протокол статуса/команд |

## Быстрый старт

### 1. Станция управления

Требуется установленный `Node.js` и `npm`.

```powershell
cd ControlStation-main
npm install
npm run dev
```

Сборка production-версии:

```powershell
cd ControlStation-main
npm run build
```

Полезные файлы:

- `ControlStation-main/package.json` - npm-скрипты и зависимости;
- `ControlStation-main/README.md` - карта `request id` и правила обмена ответами.

### 2. Прошивки микроконтроллеров

Для всех firmware-проектов используется `PlatformIO`.

Сборка выполняется из каталога конкретного модуля:

```powershell
cd compute_mcu_firmware
pio run
```

```powershell
cd prizm_firmware
pio run
```

```powershell
cd stepper_mcu_firmware
pio run
```

## Требования

- `PlatformIO Core` или `PlatformIO IDE`;
- `Node.js` и `npm` для `ControlStation-main`;
- программатор `USBasp` для окружений с "голым" `ATmega328P`, где прошивка заливается без bootloader;
- для `prizm_firmware` нужен внешний каталог библиотек `..\LigaRobotMainBoard\lib`, так как он подключён через `lib_extra_dirs` в `platformio.ini`.

Ожидаемая раскладка для сборки `prizm_firmware`:

```text
GitHub/
  robot-aix/
  LigaRobotMainBoard/
```

## Точки входа

- `ControlStation-main/src/main.tsx` - запуск фронтенда;
- `prizm_firmware/src/main.cpp` - основная прошивка PRIZM;
- `compute_mcu_firmware/src/main.cpp` - основная прошивка compute-контроллера;
- `stepper_mcu_firmware/src/main.cpp` - основная прошивка шагового контроллера.

## Дополнительные материалы

- `stepper_mcu_firmware/_codex/` содержит рабочие заметки по архитектуре, плану тестирования и ходу реализации;
- `tests/` внутри подпроектов содержит локальные тесты и экспериментальные сценарии, где они уже добавлены.

## Текущее состояние

Репозиторий находится в активной разработке. На данный момент:

- базовая многоузловая архитектура уже выделена по отдельным подпроектам;
- для прошивок реализован рабочий минимум Stage 1;
- в части модулей уже подготовлены точки расширения для следующих этапов;
- корневой README описывает общую картину, а детальные протоколы и ограничения вынесены в README и исходники конкретных подпроектов.
