

- start tmux
- on one console, start the ps3 bluetooth connection:
   sudo bash
   rosrun ps3joy ps3joy.py
- Press the P button on the ps3 controller.
- On another console, start the rest of the system:
   roslaunch boxy_bringup flatbase.launch 


