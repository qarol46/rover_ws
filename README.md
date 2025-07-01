# Проект: ТРК-211

Система управления и навигации для робототехнической платформы, включающая обработку программный модуль с симуляцией, эмуляцию движения реального робота, слияние колёсной одометрии и IMU, картирование. Основана на ROS2 Humble.

---

## Оглавление
1. [Содержимое проекта](#содержимое-проекта)
2. [Сценарии запуска](#сценарии-запуска)
3. [Установка](#установка)

---

## Содержимое проекта

| Пакет                          | Назначение                                                                |
|--------------------------------|---------------------------------------------------------------------------|
| `bluespace_ai_xsens_ros_mti`   | Драйвер для работа с датчиками Xsens MTi (ориентация/движение)            |
| `echo_server`                  | Сервер для эмуляции работы реального робота                               |
| `motion_emulator`              | Скрипты команд для проезда на 1 м/поворота на 90 градусов                 |
| `odometry_fus`                 | Слияние колёсной одометрии и данных с иму, фильтр частиц                  |
| `real_rover`                   | Управление реальным роботом                                               |
| `ros2_control_wheeled_robot`   | Контроллер дифференциального привода ros2_contorol                        |
| `rover_description`            | Описание модели робота и датчиков,запуск симуляции                        |
| `rover_navigation`             | Навигация через SLAM                                                      |
| `rover_rtabmap`                | RTAB-Map                                                                  |
| `velodyne_simulator`           | Симулятор лидара Velodyne                                                 |

---

## Сценарии запуска

### `Запуск плагина ros2_control для отладки системы управленяи колёсным шасси`
**Подготовка к запуску**:
1. Убедитесь, что в файле ros2_control.xacro параметры udp_ip, udp_port, local_port содержат значения 127.0.0.1, 8889, 8888 соответственно.
2. Закоментируйте в файле запуска launch_rover ноды, связынные с запуском IMU, лидара и слиянием данных(xsens_launch, odometry_fus_node, VLP_driver, VLP_pointcloud, translate).

**Порядок запуска**:
1. Выполните сборку пакетов, активируйте окружение и запустите эхо-сервер, воспроизводящий обратную связь с робота.
```bash
colcon build --symlink-install

source install/setup.bash

ros2 run echo_server plat_part
```
2. Перейдя в другой терминал, выполните активацию окружения и запустите файл.
```bash
source install/setup.bash

ros2 launch real_rover launch_rover.launch.py
```
3. Для управления колёсным шасси можно воспользоваться любым из способов передать управляющие команды в тему /diff_cont/cmd_vel. Например,
```bash
source install/setup.bash

ros2 launch real_rover teleop.launch.py #при использовании джойстика

ros2 run teleop_twist_keyboard teleop_twist_keyboard #при использовании клавиатуры
```

**Возможные поломки и их исправления**:
1. Сообщение "No wheel data recieved" от [Wheeled_robot_hardware] - проверьте корректность указанных значений в параметрах udp-протокола, перезапустите файл запуска launch_rover.launch.py.
2. Отсутствие движение робота - проверьте наличие связи между echo_server и wheeled_robot_hardware, налиичие команд, публикующихся в тему /diff_cont/cmd_vel. 


### `Запуск симуляции в среде Gazebo`

**Порядок запуска**:
1. Выполните сборку пакетов, активируйте окружение и запустите файл симуляции.
```bash
colcon build --symlink-install

source install/setup.bash

ros2 launch rover_description launch_sim.launch.py
```
2. Перейдя в другой терминал, выполните активацию окружения и запустите файл запуска slam.
```bash
source install/setup.bash

ros2 launch rover_navigation slam.launch.py use_sim_time:=true
```
3. Для управления колёсным шасси выполните:
```bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### `Запуск на реальном роботе`
**Подготовка к запуску**:
1. Убедитесь, что в файле ros2_control.xacro параметры udp_ip, udp_port, local_port содержат корректные значения, соответствующие реальному роботу.
2. Проверьте натройки сети вашего устройства для получения данных от платформы и лидара.
3. Проверьте работу драйвера Xsens для портирования данных в ROS2.

**Порядок запуска**:
1. Выполните сборку пакетов, активируйте окружение и запустите файл.
```bash
colcon build --symlink-install

source install/setup.bash

ros2 launch real_rover launch_rover.launch.py
```
2. Перейдя в другой терминал, выполните активацию окружения и запустите следующий файл для построения карты.
```bash
source install/setup.bash

ros2 launch rover_navigation slam.launch.py
```
3. Для управления колёсным шасси выполните:
```bash
source install/setup.bash

ros2 launch real_rover teleop.launch.py
```

**Возможные поломки и их исправления**:
1. Отсутсвие отклика на поворот - проверьте, публикует ли тема /imu/data данные. Если нет, сперва проверьте кореектность указанного топика в odometry_fus.yaml, затем проверьте корректность работы драйвера для портирования данных от Xsens в ROS2.
2. Сообщение "No wheel data recieved" от [Wheeled_robot_hardware] - проверьте корректность указанных значений в параметрах udp-протокола, перезапустите файл запуска launch_rover.launch.py.

## Установка

### 1. Клонирование репозитория

```bash
mkdir -p ~/ros2_ws/src  # Создание рабочего пространства
cd ~/ros2_ws/src
git clone https://github.com/qarol46/rover_ws.git
cd ~/ros2_ws
```

### 2. Сборка

```bash
# Установка зависимостей (из корня рабочего пространства)
rosdep install --from-paths src --ignore-src -r -y

# Сборка пакетов
colcon build --symlink-install

# Активация окружения
source install/setup.bash
```
