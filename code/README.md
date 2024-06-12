# ECE276B PR2 Spring 2024

## Overview
```
│  main.py # run the code here
│  README.md
│
│
├─maps # contains all maps
│
├─rrt_algorithms # python packeage from https://github.com/motion-planning/rrt-algorithms
│  
├─utils
│  │  __init__.py
│  │  collision_detection.py # part1: AABBs and line segment collision detection
│  │  astar.py # part2: implement astar algorithm
│  │  rrt.py # part3: modify the RRT algorithm from existing packages for use in our environments
│  │  Planner.py # baseline planner
```

## Part 1

collision_detection.py , implement collision checking with high efficiency

## Part 2

astar.py implements the A* algorithm with support for adaptive resolution. The parameters provided are universally applicable; detailed tuning is documented in the report.

## Part 3

astar.py modify the RRT algorithm from existing packages for use in our environments . The parameters provided are universally applicable; detailed tuning is documented in the report.

