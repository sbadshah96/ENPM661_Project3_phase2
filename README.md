## a_star_shreejay_aaqib_phase2.py

Shreejay Badshah (119224564 - sbadshah)\
Aaqib Barodawala (119348710 - abarodaw)

This python3 script performs A* search and planning algorithm on a 600cmx200cm canvas with obstacles

The scripts includes following libraries:\
-math\
-heapdict\
-numpy\
-vidmaker\
-sortedcollections\
-time\
-pygame

**To install heapDict():**
Run below command in the terminal:
-pip install HeapDict

**To install sortedcollections():**
Run below command in the terminal:
-pip install sortedcollections

**To install vidmaker():**
Run below command in the terminal:
-pip install vidmaker

To run the code, open any editor (preferably VSC) and run the script.

***Testing part01***\
To test the A* algorithm in 2D.
1. Run the python script and provide obstacle buffer value (Example: 2)
2. Provide initial x, y and theta values with spaces (Example: 10 95 5)
3. Provide goal x, y and theta values with spaces (Example: 170 220 90)
4. Provide 2 RPM values, the script will generate 8 action sets based on RPM values
5. Search algorithm will provide a backtrack path along with time value it took to search the goal node.
6. The animation video of tracking will be generated and saved as an mp4 file.

***Testing part02***\
To test the A* algorithm in 3D.
1. Change directory to "catkin_ws".
```
cd catkin_ws
```
2. Build all packages.
```
catkin_make
```
3. Launch the Gazebo simulation that runs A*. 
```
roslaunch a_star a_star.launch
```

***git link***\
https://github.com/sbadshah96/ENPM661_Project3_phase2.git

***Animation link***\
https://drive.google.com/file/d/1NewaH_FyrsMTiX6cgzDbv95jZTngOdvn/view?usp=sharing

