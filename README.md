# geometric-parallel-parking

Geometric path planning for automatic parallel parking in tiny spots [1]

## RUN
`python main.py --x_start 75 --y_start 40 --psi_start -20 --x_end 90 --y_end 80 --parking_length 12 --parking_margin 1`

### One Traial
https://user-images.githubusercontent.com/39142679/175810486-3b592e5a-864d-42a8-a498-1e65b36e1303.mp4

https://user-images.githubusercontent.com/39142679/175810488-b173f3bd-0f38-4188-8f55-56d01a171d80.mp4

### Several Trarial
https://user-images.githubusercontent.com/39142679/175810944-a3557d76-9503-4539-b68a-761a4abecedf.mp4

https://user-images.githubusercontent.com/39142679/175810980-01661de2-08cd-4499-a2df-240160ddaf43.mp4

## Application

[Autoware](https://github.com/autowarefoundation/autoware) uses C++ version of this path planning. [geometric_parallel_parking.cpp](https://github.com/autowarefoundation/autoware.universe/blob/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/utils/parking_departure/geometric_parallel_parking.cpp) is implemented in Autoware which is used in [goal_planner](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/#geometric-pull-out) and [start_planner](https://autowarefoundation.github.io/autoware.universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/#geometric-pull-out).

https://github.com/autowarefoundation/autoware.universe/assets/39142679/0ddf94d5-11b1-4dff-a0fe-a1237c632191

## Reference
based on https://github.com/Pandas-Team/Automatic-Parking and replaced the path planner.

[1] H. Vorobieva, S. Glaser, N. Minoiu-Enache, and S. Mammar, “Geometric path planning for automatic parallel parking in tiny spots,” IFAC Proceedings Volumes, vol. 45, no. 24, pp. 36–42, 2012.
