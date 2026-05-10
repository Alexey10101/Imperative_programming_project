# robot_spatial

Console utility in C for 2D/3D spatial data processing from CSV:
- `k-d tree`: insert, delete, nearest neighbor
- `Fuzzy C-means`: soft clustering (centroids, hard labels, membership degrees)
- `DBSCAN`: density-based clustering with noise detection

## Requirement Coverage

- Language: **C**
- Spatial data support: **strictly 2D or 3D**
- `k-d tree` operations:
  - point insertion
  - point deletion
  - nearest-neighbor search
- `Fuzzy C-means`:
  - iterative centroid update
  - membership matrix computation
- `DBSCAN`:
  - epsilon-neighborhood search
  - cluster expansion
  - noise marking (`-1`)

## Build

Linux/macOS:

```bash
make
```

or:

```bash
gcc -O2 -std=c11 -Wall -Wextra -pedantic robot.c -lm -o robot_spatial
```

Windows (MinGW):

```powershell
gcc -O2 -std=c11 -Wall -Wextra -pedantic robot.c -lm -o robot_spatial.exe
```

## Input Format

CSV without header, one point per line:

```csv
0.0,1.0
1.2,3.4
5.6,7.8
```

Rules:
- all rows must have the same dimensionality;
- only `x,y` or `x,y,z` are allowed.

## Usage

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

Examples:

```bash
./robot_spatial lidar.csv -kd_nearest 1.0,2.0
./robot_spatial lidar.csv -cmeans 3
./robot_spatial lidar.csv -dbscan 0.5,5
```

PowerShell note:

```powershell
.\robot_spatial.exe lidar.csv -dbscan '0.5,5'
```
