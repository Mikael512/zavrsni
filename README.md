# zavrsni
1) git clone https://github.com/Mikael512/zavrsni.git
2) catkin build cijeli moveit_ws paket
3) cd move_it/
4) source devel/setup.bash
5) roslaunch zavrsni program1.launch gazebo:=false


problem:
Ako zovemo naredbu $roslaunch zavrsni program1.launch gazebo:=true
oba launch fajla se izvrse bez error, ali na kraju kada stisnemo enter za isplanirati i izvrsiti trajektoriju, rviz javlja "no environment"
osim toga u rvizu mozemo vidjeti u global kartici da nema TF data tj pise "no TF data"
...rješeno
treba samo pokrenuti simulaciju gazebo play botunom.


