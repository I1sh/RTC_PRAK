# ROS пакет для работы с лазерным дальномером ls2d

## Подключение датчика

Подлючение сенсора осуществляется в соответсвии с [его документацией](http://prizmasensors.ru/ls2d-triangulyacionnyj-lazernyj-2d-datchik/) (раздел "ссылки", Техническое описание LS2D). Основные пункты, на которые стоит обратить внимание:

- Мощность датчика
- Поляронсь подлкючения
- Параметры IPv4

## Запуск пакета

### Запуск в контейнере docker

1. Установите [docker](https://docs.docker.com/engine/install/ubuntu/) при необходимости.
Склонируйте репозиторий и перейдите в его основную папку:

```shell
git clone https://github.com/I1sh/RTC_PRAK.git
```

2. Для работы с docker-окружением доступны следующие скрипты:

```shell
docker/build_docker.sh - сборка контейнера
docker/run_docker.sh - запуск контейнера
docker/into_docker.sh - запуск bash в контейнерере
```

В случае, если на вашем ПК используется видеокарта от nvidia, то скрипты сборки и запуска следует исполнять с параметром `-n` или `--nvidia`.

Соответственно, для начала работы соберите и запустите контейнер выполнив последовательно следующие команды:

```shell
bash <path-to-this-rep>/docker/build_docker.sh
bash <path-to-this-rep>/docker/run_docker.sh
```

3. При первом входе в контейнер требуется собрать пакеты в директории `catkin_ws`, для этого внутри контейнера выполните:

```shell
cd /catkin_ws
catkin_make 
```

### Запуск вне контейнера

1. Склонируйте репозиторий и перенесите пакет в ваше окружение:

```shell
git clone https://github.com/I1sh/RTC_PRAK.git
mv RTC_PRAK/catkin_ws/src/ls2d <path-to-your-ws/src>
```

2. Установите библиотеку [boost](https://www.boost.org/):
```shell 
sudo apt-get install libboost-all-dev
```

3. Перейдите в ваш воркспейс и соберите пакет:

```shell
rosdep install --from-paths src --ignore-src -r -y
catkin_make 
source devel/setup.bash
```

## LS2D ros package

Пакет содержит ноду `ls2d_init_node`, которая подлючается к датчику и публикует данные с датчика в топик `point_cloud`, тип сообщения `sensor_msgs/PointCloud2.msg`. Данные об иницализации датчика публикуются в топик `sensor_info`, тип сообщения `ls2d/Sensor_info.msg`.

Для запуска всех нижеописаных пакетов одной командой можно воспользоваться launch файлом:

```shell
TODO
```

---

1. Для запуска визуализации данных в rviz нужно оперделить tf для фрейма датчика `/sensor_base_link`. При запуске датчика отдельно можно воспользоваться командой: 

```shell
rosrun tf static_transform_publisher 0 0 0 0 0 0 map sensor_base_link 50
```

В общем случае же необходимо определить `static_tf` между роботом и датчиком.

2. Для преобразования получаемого с датчика облака точек к сообщению типа `sensor_msgs/LaserScan.msg` нужно запусить лаунч файл point_cloud_to_lazerscan.launch из этого пакета. При необходимости в нём можно задать имя фрейма сенсора, параметры сканирования (высоту, ширину, угол сканирования), названия топиков для входого облака точек (по умолчанию /point_cloud) и выходного сообщения laserscan(по умолчанию /scan_msgs).

```shell
roslaunch ls2d point_cloud_to_lazerscan.launch
```

3. В rviz необходимо добавить визуализацию LaserScan и указать имя топика из которого чиать сообщеня соответсвующего типа.

### License

License: [APACHE LICENSE, VERSION 2.0](http://www.apache.org/licenses/LICENSE-2.0).

**Author: Ivan Shevtsov ishevtsov0108@gmail.com **

Пакет тестировался и разробатывался под [ROS] Noetic и ubuntu 20.04.