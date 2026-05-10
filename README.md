# robot_spatial

Консольная утилита на C для обработки пространственных данных 2D/3D из CSV:
- `k-d tree`: вставка, удаление, поиск ближайшего соседа
- `Fuzzy C-means`: мягкая кластеризация (центроиды, hard-метки, степени принадлежности)
- `DBSCAN`: кластеризация по плотности с выделением шума
Разбор:
- `k-d tree`:
  - добавление точки
  - удаление точки
  - поиск ближайшей точки
- `Fuzzy C-means`:
  - итеративное обновление центроидов
  - вычисление степеней принадлежности
- `DBSCAN`:
  - поиск точек в `ε`-окрестности
  - расширение кластера
  - шум помечается как `-1`

## Сборка

Linux/macOS:

```bash
make
```

или:

```bash
gcc -O2 -std=c11 -Wall -Wextra -pedantic robot.c -lm -o robot_spatial
```

Windows (MinGW):

```powershell
gcc -O2 -std=c11 -Wall -Wextra -pedantic robot.c -lm -o robot_spatial.exe
```

## Формат входных данных

CSV без заголовка, одна точка в строке:

```csv
0.0,1.0
1.2,3.4
5.6,7.8
```

Требования:
- во всех строках одинаковое число координат;
- допустимы только `x,y` или `x,y,z`.

## Использование

```bash
./robot_spatial <csv> -kd_insert  x,y
./robot_spatial <csv> -kd_insert  x,y,z
./robot_spatial <csv> -kd_delete  x,y
./robot_spatial <csv> -kd_delete  x,y,z
./robot_spatial <csv> -kd_nearest x,y
./robot_spatial <csv> -kd_nearest x,y,z
./robot_spatial <csv> -cmeans <clusters> [m] [max_iter] [tol]
./robot_spatial <csv> -dbscan <eps,minPts>
./robot_spatial <csv> -dbscan <eps> <minPts>
```

Примеры:

```bash
./robot_spatial lidar.csv -kd_nearest 1.0,2.0
./robot_spatial lidar.csv -cmeans 3
./robot_spatial lidar.csv -dbscan 0.5,5
```

Для PowerShell:

```powershell
.\robot_spatial.exe lidar.csv -dbscan '0.5,5'
```
