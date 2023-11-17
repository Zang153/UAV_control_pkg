
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS2:921600"; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch vrpn_client_ros sample.launch server:=192.168.1.107; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command square.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun px4_command move; exec bash"' \

