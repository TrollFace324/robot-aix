# robot-aix

## Описание проекта

**robot-aix** — монорепозиторий интегрированной системы управления роботом с трёхуровневой архитектурой микроконтроллеров. Система обеспечивает управление серво-приводами, исполнительными механизмами и сбор телеметрии через веб-интерфейс оператора.

Проект объединяет:

- **Веб-интерфейс станции управления** — реактивный фронтенд на `Vite + React + TypeScript`;
- **Прошивку основного контроллера PRIZM** — главный координатор с поддержкой режимов управления и автономных операций;
- **Прошивку вычислительного контроллера (Compute MCU)** — промежуточный узел обработки данных и маршрутизации протоколов;
- **Прошивку шагового контроллера** — специализированный узел управления исполнительными механизмами по I2C.

---

## Архитектура системы

### Общая структура

```
┌─────────────────────────────┐
│  Control Station            │
│  (Web UI: Vite + React)     │
│  Оператор управляет через   │
│  интерфейс с кнопками       │
└──────────────┬──────────────┘
               │ HC12 (RF Link)
               ↓
┌──────────────────────────────────────┐
│  PRIZM Controller (Main MCU)         │
│  • Координирует режимы управления    │
│  • Управляет серво-приводами        │
│  • Обработка входных команд          │
└──────────────┬──────────────────────┘
               │ SPI (32-byte frames)
               ↓
┌──────────────────────────────────────┐
│  Compute MCU (ATmega328P)            │
│  • Маршрутизация протоколов          │
│  • Самодиагностика (Self-check)      │
│  • IMU (MPU6500) интеграция          │
└──────────────┬──────────────────────┘
               │ I2C (24-byte frames)
               ↓
┌──────────────────────────────────────┐
│  Stepper Controller (ATmega328P)     │
│  • Управление шаговыми моторами      │
│  • Обработка команд HOME/GOTO        │
│  • Отчёт о положении и ошибках       │
└──────────────────────────────────────┘
```

### Транспортные системы

| Уровень | Протокол | Размер кадра | Компоненты | Назначение |
|---------|----------|--------------|-----------|-----------|
| **HCI** | HC12 RF | Переменный | Control Station ↔ PRIZM | Беспроводная связь оператор-робот |
| **SPI** | CRC16 (CCITT-FALSE) | 32 байта | PRIZM ↔ Compute | Синхронный обмен, реальное время |
| **I2C** | CRC8 (poly: 0xD5) | 24 байта | Compute ↔ Stepper | Адресная линия подвижных механизмов |

---

## Структура репозитория

### Основные компоненты

| Каталог | Описание | Статус | Язык |
|---------|---------|--------|------|
| **`ControlStation-main/`** | Веб-интерфейс оператора | ✅ Функционален | TypeScript/React |
| **`prizm_firmware/`** | Главный контроллер системы | 🔄 Stage 1 готов, Stage 2/3 в разработке | C++ (PlatformIO) |
| **`compute_mcu_firmware/`** | Вычислительный узел | ✅ Stage 1 стабилен | C++ (PlatformIO) |
| **`stepper_mcu_firmware/`** | Контроллер шаговых моторов | ✅ Ядро готово | C++ (PlatformIO) |

### Подробное описание каждого компонента

#### 1. **ControlStation-main** — Веб-интерфейс станции управления

**Назначение:** Пользовательский интерфейс для оператора. Обеспечивает управление всеми функциями робота через браузер.

**Технологический стек:**
- `Vite` — современный build-инструмент
- `React 18` — UI-фреймворк с хуками
- `TypeScript` — типизированный JavaScript
- `CSS Modules` — изолированные стили (LiquidChrome)

**Структура:**
```
ControlStation-main/
├── src/
│   ├── App.tsx              # Главный компонент приложения
│   ├── main.tsx             # Точка входа
│   ├── kranActionState.ts   # State management для манипулятора
│   ├── LiquidChrome.tsx     # UI компонент с жидким дизайном
│   ├── telemetryBoomEditor.ts # Редактор телеметрии стрелы
│   └── store/               # Redux/Context store
├── tests/                   # Тесты компонентов
├── assets/                  # Статические ресурсы
├── package.json             # Зависимости и скрипты
├── tsconfig.json            # Конфигурация TypeScript
└── vite.config.d.ts         # Конфигурация Vite
```

**Ключевые возможности:**
- Навигация между вкладками (Main, Kran, Lapa, Servo, Telemetry, Label)
- Управление манипулятором (режимы Авто/Ручной, движения)
- Управление захватом (лапы, захватить/отпустить)
- Управление серво-приводами (выбор, движение, позиционирование)
- Система запросов с идентификаторами (request ID) и ответами
- Автоматическое поддержание соединения (каждые 10 секунд keep-alive запрос)

**Протокол взаимодействия:**

Формат запроса: `REQ,<id>` (для специальных операций: `REQ,<id>,<servoNumber>`)

Формат ответа: `RESP,<id>` или двухпакетный: `RESP,<id>,0` → `RESP,<id>,1`

Специальные ID с двухпакетным ответом (требуют блокировки UI):
- `6` — Старт автономного режима
- `10` — Стрелка вниз (Kran)
- `11` — Кнопка «Исходное» (Kran)
- `12` — Кнопка «Выбросить» (Kran)
- `16` — Кнопка «Захватить» (Lapa)
- `17` — Кнопка «Отпустить» (Lapa)
- `20-24` — Servo команды с номером привода

**Быстрый старт:**

```powershell
cd ControlStation-main
npm install
npm run dev          # Запуск dev-сервера на http://localhost:5173
npm run build        # Сборка production
npm run preview      # Просмотр собранной версии
npm run test         # Запуск тестов
```

---

#### 2. **prizm_firmware** — Основной контроллер PRIZM

**Назначение:** Главный координатор системы. Получает команды от оператора, управляет режимами работы, координирует подчинённые контроллеры.

**Аппаратная платформа:**
- TETRIX PRIZM микроконтроллер
- CRSF receiver для RF-связи (HC12 модуль)
- SPI мастер для связи с Compute MCU
- Встроенные драйверы серво и моторов

**Структура прошивки:**
```
prizm_firmware/
├── src/
│   ├── main.cpp             # Точка входа
│   ├── app/                 # Основная логика приложения
│   │   ├── compute_app.h    # Управление Compute MCU
│   │   └── ...
│   ├── station/             # Взаимодействие со станцией
│   │   └── hc12_backend.*   # HC12 RF транспорт
│   ├── protocol/            # Протоколирование, SPI-мастер
│   │   └── spi_master_backend.*
│   ├── autonomous/          # Режим автономного управления
│   ├── control/             # Управление режимом
│   ├── drive/               # Управление моторами
│   ├── safety/              # Проверки безопасности
│   ├── status/              # Отчётность о статусе
│   └── radio/               # Радио-протоколы
├── config/
│   ├── pin_map_prizm.h      # Распиновка
│   ├── debug_config.h       # Debug-параметры
│   ├── timing_config.h      # Таймауты и интервалы
│   └── safety_config.h      # Лимиты и проверки
├── include/
│   ├── protocol_types.h     # Типы сообщений
│   ├── system_state.h       # Состояние системы
│   └── transport_if.h       # Интерфейсы транспорта
├── lib/
│   ├── legacy_liga_adapter/ # Переиспользуемые компоненты
│   └── TETRIX_PRIZM/        # SDK PRIZM
├── _codex/                  # Документация архитектуры
│   ├── architecture.md
│   ├── protocols.md
│   ├── todo.md
│   └── ...
└── platformio.ini           # PlatformIO конфигурация
```

**Текущий статус разработки:**

**Stage 1 (✅ Завершён):**
- SPI транспорт с PRIZM как мастер
- Основные протокольные сообщения: HELLO, STATUS, CAPABILITIES, HEARTBEAT
- Управление режимами (Авто/Ручной)
- Интеграция с HC12 RF-модулем

**Stage 2 (🔄 В разработке):**
- Heading hold (удержание курса)
- Расширенный режим управления
- Сохранение состояния в EEPROM

**Stage 3 (📋 Планируется):**
- Mission runner (выполнение программ автономных операций)
- Advanced autonomy behaviors
- Логирование в памяти

**Ключевые параметры:**
```cpp
// timing_config.h
#define HC12_BAUD_RATE 115200        // Скорость HC12
#define SPI_CLOCK_RATE 1000000       // 1 МГц SPI
#define SELF_CHECK_TIMEOUT_MS 5000   // Таймаут самодиагностики
```

**Сборка и прошивка:**

```powershell
cd prizm_firmware
pio run                      # Собрать
pio run -t upload            # Загрузить в контроллер
pio run -e debug -t upload   # Debug-версия
pio check                    # Проверка синтаксиса
```

---

#### 3. **compute_mcu_firmware** — Вычислительный контроллер

**Назначение:** Промежуточный узел обработки данных. Маршрутизирует команды между PRIZM и Stepper MCU, проводит диагностику (IMU, связь), управляет режимами запуска.

**Аппаратная платформа:**
- ATmega328P микроконтроллер
- MPU6500 IMU (6-осная инерциальная система)
- SPI-slave интерфейс для PRIZM
- I2C-master интерфейс для Stepper MCU

**Структура прошивки:**
```
compute_mcu_firmware/
├── src/
│   ├── main.cpp             # Точка входа
│   ├── app/
│   │   └── compute_controller.* # Главный контроллер маршрутизации
│   ├── protocol/            # Обработка протоколов
│   │   ├── spi_slave_backend.* 
│   │   └── i2c_stepper_client.*
│   ├── imu/                 # MPU6500 интеграция
│   │   └── mpu6500_service.*
│   ├── diag/                # Диагностика и самопроверка
│   │   └── self_check_runner.*
│   ├── autonomous/          # Logics for autonomous ops
│   ├── station/             # (Если требуется прямая связь)
│   └── transport/           # Транспортные абстракции
├── config/
│   ├── pin_map_compute.h    # Распиновка Compute MCU
│   ├── imu_config.h         # Параметры MPU6500
│   ├── transport_config.h   # Параметры SPI/I2C
│   └── debug_config.h
├── include/
│   ├── compute_app.h        # Public interface
│   ├── protocol_types.h
│   ├── health_report.h      # Структура отчёта о здоровье
│   ├── imu_if.h
│   ├── stepper_link_if.h
│   └── transport_if.h
├── lib/
│   └── robot_protocol/      # Общие типы протокола
├── _codex/                  # Документация
│   ├── architecture.md
│   ├── protocols.md
│   ├── implemented.md
│   └── ...
├── tests/                   # Модульные и интеграционные тесты
│   ├── auto_home_session_test.cpp
│   ├── imu_settings_store_test.cpp
│   ├── kran_fsm_test.cpp
│   └── self_check_runner_test.cpp
└── platformio.ini
```

**Ключевые компоненты:**

1. **SPI Slave Backend** — Приёмо-передатчик SPI кадров
   - Фиксированный размер кадра: 32 байта
   - Header: SOF(1) / VER(1) / MSG(1) / FLAGS(1) / SRC(1) / DST(1) / SEQ(1) / ACK_SEQ(1) / LEN(1)
   - Payload: до 20 байт
   - Checksum: CRC16 (CCITT-FALSE) на байты [0..28]

2. **Compute Controller** — Маршрутизатор протоколов
   - Обрабатывает входящие SPI команды
   - Направляет их в I2C Stepper Client или обрабатывает локально
   - Управляет режимами (INIT, READY, RUNNING, ERROR)

3. **Self-Check Runner** — Диагностика при запуске
   - Проверка IMU
   - Проверка связи со Stepper MCU
   - Генерация READY-сигнала только при успешной диагностике

4. **I2C Stepper Client** — Нижнеуровневый транспорт к шаговому контроллеру
   - 24-байтные кадры с SOF = 0x5A
   - CRC8 (polynomial 0xD5)
   - Команды: HOME, GOTO, EXTEND, RETRACT, STOP, SET_LIMITS, REQUEST_STATUS

5. **MPU6500 Service** — Интеграция IMU
   - Инициализация сенсора при запуске
   - Probe-проверка для self-check
   - Буферизация сырых данных

**Протоколы:**

**SPI (PRIZM ↔ Compute):**
```
Сообщения:
- HELLO / HELLO_ACK        — Инициализация связи
- GET_STATUS / STATUS      — Запрос и отчёт о статусе
- GET_CAPABILITIES / CAPABILITIES — Возможности ноды
- HEARTBEAT / HEARTBEAT_ACK — Проверка живости
- STARTUP_CHECK_RESULT — Результат самодиагностики
- READY_TO_RUN / RUN_LOCKED — Управление режимом запуска
- ECHO_TEST / TEST_FLAG — Тестирование связи
```

**I2C (Compute ↔ Stepper):**
```
Команды:
- HOME              — Инициализация с обнулением
- GOTO <position>   — Переместиться в позицию
- EXTEND            — Максимальное расширение
- RETRACT           — Максимальное сжатие
- STOP              — Аварийная остановка
- SET_LIMITS        — Установка лимитов хода
- REQUEST_STATUS    — Запрос текущего состояния

Статус:
- position, target position
- homed flag
- driver state
- home switch status
- fault flags (overcurrent, thermal, etc.)
```

**Статус разработки:**

**Stage 1 (✅ Стабилен):**
- SPI slave транспорт с ISR обработкой
- Compute controller маршрутизация
- Self-check интеграция (IMU + Stepper link)
- I2C stepper client с CRC8

**In Progress (🔄):**
- Bench-тестирование SPI slave timing под нагрузкой
- Полный IMU data path (за пределами startup health check)
- Дублирование/стейл-пакет стратегия под нагрузкой

**Blocking Issues (⚠️):**
- Точная распиновка платы ожидается
- Константы ориентации MPU6500 в разработке

**Быстрый старт:**

```powershell
cd compute_mcu_firmware
pio run                      # Собрать
pio run -t upload            # Загрузить (требуется USBasp или встроенный bootloader)
pio test                     # Запуск unit-тестов
pio check                    # Статический анализ
```

---

#### 4. **stepper_mcu_firmware** — Контроллер шаговых моторов

**Назначение:** Специализированный контроллер для управления шаговыми моторами и исполнительными механизмами (кран, захват и т.д.).

**Аппаратная платформа:**
- ATmega328P микроконтроллер
- I2C-slave интерфейс (подчинённый)
- Драйверы шаговых моторов (DRV8825 или A4988)
- Датчики лимитов (home switch, position sensors)

**Структура прошивки:**
```
stepper_mcu_firmware/
├── src/
│   ├── main.cpp             # Точка входа
│   ├── app/                 # Основная логика
│   ├── transport/           # I2C slave backend
│   ├── control/             # Управление движением
│   ├── drivers/             # Драйверы моторов
│   └── sensors/             # Обработка датчиков
├── config/
│   ├── pin_map_stepper.h    # Распиновка
│   ├── i2c_config.h         # I2C параметры (адрес, скорость)
│   ├── kran_slot_config.h   # Конфигурация слотов крана
│   └── ...
├── include/
│   ├── protocol_types.h
│   ├── transport_if.h
│   └── ...
├── lib/                     # Внешние библиотеки
├── _codex/                  # Документация
├── tests/                   # Тесты
│   └── motors_prizm_exp_test.cpp
└── platformio.ini
```

**Ключевые возможности:**
- Управление несколькими шаговыми моторами
- Хоум-позиционирование (инициализация с датчиком home switch)
- Позиционное управление (GOTO с целевой позицией)
- Расширенные/сжатые операции (EXTEND/RETRACT)
- Лимиты хода (мягкие и жёсткие)
- Отчётность о состоянии (позиция, цель, флаг готовности, ошибки)

**I2C Protocal (24-byte frame):**
```
[SOF=0x5A][CMD][...payload...][CRC8]

Поддерживаемые команды:
- 0x01: HOME                 — Инициализация
- 0x02: GOTO <position>      — Переместиться
- 0x03: EXTEND               — Максимум вперёд
- 0x04: RETRACT              — Максимум назад
- 0x05: STOP                 — Остановка
- 0x0A: SET_LIMITS           — Установка лимитов
- 0x0F: REQUEST_STATUS       — Запрос статуса
- 0xF0: ECHO_TEST            — Эхо-тест связи
```

**Быстрый старт:**

```powershell
cd stepper_mcu_firmware
pio run                      # Собрать
pio run -t upload            # Загрузить
pio test                     # Запуск тестов
```

---

## Требования к окружению

### Обязательные

- **PlatformIO Core** (≥ 6.1.x)
  ```powershell
  pip install platformio
  ```

- **Node.js** (≥ 18.0) и **npm** (≥ 9.0) для Control Station
  ```powershell
  node --version    # Проверка Node.js
  npm --version     # Проверка npm
  ```

### Опциональные (для прошивки железа)

- **GNU ARM Embedded Toolchain** — автоматически устанавливается через PlatformIO
- **Программатор USBasp** — для систем без встроенного bootloader
- **CH340 USB-UART адаптер** — для отладки (debugging via serial)
- **Avrdude** — автоматическая прошивка через PlatformIO

### Рекомендуемые инструменты

- **VS Code** + расширение PlatformIO IDE
- **Git** (≥ 2.30) для версионирования
- **SerialMonitor** или **PuTTY** для отладки через COM-порты

---

## Быстрый старт

### Вариант 1: Запуск Control Station (веб-интерфейс)

```powershell
cd ControlStation-main
npm install
npm run dev
```

Откройте браузер: `http://localhost:5173`

### Вариант 2: Сборка всех прошивок

```powershell
# Compute MCU Firmware
cd compute_mcu_firmware
pio run

# PRIZM Firmware
cd ../prizm_firmware
pio run

# Stepper MCU Firmware
cd ../stepper_mcu_firmware
pio run
```

### Вариант 3: Полная интеграция (разработка)

```powershell
# Терминал 1: Веб-интерфейс
cd ControlStation-main
npm install
npm run dev

# Терминал 2: Мониторинг PRIZM (debug serial)
cd prizm_firmware
pio device monitor -b 115200

# Терминал 3: Мониторинг Compute MCU (debug serial)
cd compute_mcu_firmware
pio device monitor -b 9600
```

---

## Структура конфигураций

### Control Station (`ControlStation-main/`)

```javascript
// vite.config.d.ts
export default {
  server: {
    port: 5173,
    proxy: {
      '/api': 'http://localhost:3000'
    }
  }
}
```

### Firmware конфигурации

```cpp
// config/pin_map_*.h
#define PIN_SPI_MOSI     11    // PlatformIO pin numbering
#define PIN_SPI_MISO     12
#define PIN_SPI_SCK      13
#define PIN_CS           10

// config/timing_config.h
#define SPI_TIMEOUT_MS   100
#define I2C_BAUD_RATE    100000    // 100 кГц для надёжности
```

---

## Тестирование

### Unit-тесты firmware

```powershell
cd compute_mcu_firmware
pio test                           # Запуск всех тестов
pio test -- -v                     # С подробным выводом
pio test -- -f "*self_check*"      # Фильтр по имени
```

### Тесты Control Station

```powershell
cd ControlStation-main
npm run test                       # Jest тесты
npm run test -- --watch           # Watch mode
```

### Интеграционные тесты

Смотрите документацию в каждом модуле `_codex/test_plan.md`.

---

## Документация

Каждый модуль содержит собственную документацию в каталоге `_codex/`:

- **`architecture.md`** — описание архитектуры и компонентов
- **`protocols.md`** — формат протоколов и сообщений
- **`implemented.md`** — список реализованных функций
- **`todo.md`** — планы развития и TODO задачи
- **`test_plan.md`** — стратегия тестирования
- **`change_log.md`** — история изменений

**Основные документы в корне репозитория:**
- [ControlStation-main/README.md](ControlStation-main/README.md) — Карта request ID и протокол взаимодействия
- [prizm_firmware/README.md](prizm_firmware/README.md) — Специфика PRIZM контроллера
- [compute_mcu_firmware/_codex/](compute_mcu_firmware/_codex/) — Полная документация Compute MCU

---

## Workflow разработки

### Для добавления новой команды управления

1. **Define the command** в `_codex/protocols.md` соответствующего модуля
2. **Implement handler** в `src/protocol/` или `src/app/`
3. **Add tests** в `tests/`
4. **Update documentation** в `_codex/implemented.md`
5. **Test integration** через интеграционные сценарии

### Для добавления новой вкладки в Control Station

1. **Create React component** в `src/components/`
2. **Add request IDs** в документацию `ControlStation-main/README.md`
3. **Implement backend handler** в PRIZM firmware
4. **Test UI** через `npm run dev`

### Для отладки SPI/I2C связи

```powershell
# Включить DEBUG режим в config/*debug_config.h
#define DEBUG_SPI_ENABLED 1
#define DEBUG_I2C_ENABLED 1

# Собрать debug версию
pio run -e debug -t upload

# Мониторить вывод
pio device monitor -b 115200
```

---

## Решение проблем

### Проблема: Compute MCU не отвечает на SPI

**Решение:**
1. Проверьте распиновку в `config/pin_map_compute.h`
2. Убедитесь, что PRIZM установлена как SPI-мастер
3. Включите `DEBUG_SPI_ENABLED` в конфиге и проверьте логи
4. Проверьте скорость SPI (обычно 1 МГц)

### Проблема: Stepper MCU не инициализируется (нет home)

**Решение:**
1. Проверьте подключение датчика home switch (обычно на pin A0)
2. Убедитесь, что I2C адрес совпадает (стандартно 0x20)
3. Проверьте питание на драйвере мотора (DRV8825)
4. Запустите эхо-тест: отправьте ECHO_TEST и проверьте ответ

### Проблема: Control Station не подключается к роботу

**Решение:**
1. Проверьте питание и включение HC12 модуля
2. Убедитесь, что PRIZM работает (индикаторы LED)
3. Проверьте серийный номер RF-модуля (`115200` baud rate)
4. Включите логирование в frontend (`kranActionState.ts`)

### Проблема: Сборка не удаётся в Windows PowerShell

**Решение:**
```powershell
# Если "pio" не признаётся, используйте полный путь
python -m platformio run

# Или обновите PATH:
pip install --upgrade platformio
platformio --version
```

---

## Статус проекта (состояние на 17 апреля 2026)

### Завершённые компоненты

- **Control Station** — полный функционал UI
- **PRIZM Firmware (Stage 1)** — базовый контроль и коммуникация
- **Compute MCU (Stage 1)** — маршрутизация, диагностика, SPI/I2C
- **Stepper Controller** — ядро управления шаговыми моторами

### В разработке

- **PRIZM Firmware (Stage 2)** — heading hold, расширенное управление
- **Compute MCU (Bench testing)** — нагрузочные тесты SPI под высокой частотой
- **IMU full data path** — полная обработка 6-осных данных IMU

### Планируется

- **PRIZM Firmware (Stage 3)** — mission runner, autonomous behaviors
- **OTA Updates** — обновление прошивок по радио
- **Logging to flash** — сохранение логов в память контроллеров
- **Advanced telemetry** — расширенная телеметрия с историей

---

## Контрибьютинг

1. **Создайте feature branch**:
   ```bash
   git checkout -b feature/my-feature
   ```

2. **Внесите изменения** и протестируйте:
   ```powershell
   pio test        # Для firmware
   npm run test    # Для Control Station
   ```

3. **Обновите документацию** в `_codex/` или основном README

4. **Создайте Pull Request** с описанием изменений

---

## Контакты и поддержка

- **Issue Tracker**: Используйте GitHub Issues для сообщения об ошибках
- **Documentation**: Смотрите папки `_codex/` в каждом модуле
- **Code Examples**: Тесты содержат примеры использования API

---

## Лицензия

[Укажите лицензию проекта]

---

**Последнее обновление**: 17 апреля 2026
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
