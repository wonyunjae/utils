# utils
util packages ( e.g rel_to_abs , tum format converter)
### SLAM evaluation EVO 활용

https://github.com/MichaelGrupp/evo

```jsx
pip3 install evo 
// visualize trajectory result 
evo_traj tum [Trakectory.txt] -p
// visualize trajectory result as 2D dimension 
evo_traj tum [Trakectory.txt] -p --plot_mode xy
// visualize more than two trajectories (Be careful of timestamp)
evo_traj tum [Trakectory1.txt] [Trakectory2.txt] -p
// Compare trajectory with GT
evo_traj tum [Trakectory.txt]  -p -a --ref [GT.txt]
//Calculate APE with 
evo_ape tum [GT.txt] [Trakectory.txt] -vap
//Convert ros bag as TUM format (Only ROS1)
evo_traj bag [Trajectory.bag] [Trajectory Topic name] --save_as_tum

```
```jsx
cd ~/[workspace_name]/src/utils/utils/tum
evo_traj tum glim_utm.txt liosam_utm.txt -p -a --ref gps_utm.txt
![image](https://github.com/user-attachments/assets/be249847-7514-4ca4-bb61-4757290035b2)

```

### Convert Odometry(Relative) -> UTM(Absolute)
input : Odometry MessageType, 기준이되는 gps/fix(NavSatfix MessageType)

Output : PointStamped MessageType (UTM Coordinate)
```jsx
ros2 run utils rel_to_abs
```
