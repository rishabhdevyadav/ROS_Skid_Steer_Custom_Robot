1. Go to Folder RobotGUI/WebPage
2. Run rcontrol.py (Install dependencies like Flask...)
3. Open a chrome/mozilla new page
4. open localhost:5000
5. Login by entering Username: "aarg", Password: "aarg"
6. Skip the welcome page (It is complete, but I need to explain you to run this)
7. Check the Speed Control (It should run a robot with the selected robot and set command velocities):
   A "cmd_vel.json" file is created in RobotGUI/config. Use the content of this json file to run the robot. 
8. Check the Piston (It should move the piston of specified robots (you can select multiple robots)):
   A "piston.json" file is created in RobotGUI/config. Use the content of this file to move the piston.
9. Check the Go To Goal (It should take a specified robot to the goal position from start position):
   A "g2g.json" file is created in RobotGUI/config. Use the content of this json file to make the robot to reach the goal.