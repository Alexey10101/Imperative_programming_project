# robot_spatial

Консольная утилита для работы с пространственными точками из CSV:
- `k-d tree`: вставка, удаление, ближайший сосед
- `Fuzzy C-means`: мягкая кластеризация
- `DBSCAN`: кластеризация по плотности

## Сборка

```bash
gcc -O2 -std=c11 robot_spatial.c -lm -o robot_spatial
```

Для Windows (MinGW):

```powershell
gcc -O2 -std=c11 robot_spatial.c -lm -o robot_spatial.exe
```

## Формат входного файла

CSV без заголовка, одна точка в строке:

```csv
0.0,1.0
1.2,3.4
5.6,7.8
```

Поддерживаются 2D/3D/… данные (k измерений), главное — одинаковое число координат в каждой строке.

## Использование

```bash
./robot_spatial <csv> -kd_insert  x,y[,z,...]
./robot_spatial <csv> -kd_delete  x,y[,z,...]
./robot_spatial <csv> -kd_nearest x,y[,z,...]
./robot_spatial <csv> -cmeans <clusters> [m=2.0] [max_iter=100] [tol=1e-4]
./robot_spatial <csv> -dbscan <eps,minPts>
./robot_spatial <csv> -dbscan <eps> <minPts>
```

Примеры:

```bash
./robot_spatial lidar.csv -kd_nearest 1.0,2.0
./robot_spatial lidar.csv -cmeans 3
./robot_spatial lidar.csv -dbscan 0.5,5
```

PowerShell: аргумент с запятой лучше передавать в одинарных кавычках:

```powershell
.\robot_spatial.exe lidar.csv -dbscan '0.5,5'
```
