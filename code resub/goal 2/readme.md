## Goal 2: Make a grid
- You may have noticed that the Turtle stops moving if you stop publishing velocity. In real life there is a deceleration profile for every vehicle. Even applying the brakes does not usually result in vehicles stopping at the current
stop. Since the turtlesim does not provide this deceleration profile, let us add it from outside.
- In your code, ensure that there is a limitation on maximum acceleration and deceleration. You can use separate parameters for each and vary them independent of each other.
- Check if you need to change the PID gains to accomplish goal 1 once again.
- Use this relatively realistic robot, and make a grid pattern as shown in the image below. You can ignore the curve formed by the turtle in the image.
- Capture the video for varying profiles of acceleration and deceleration.
- Do you need to change gains every time? If yes/no, why?
- Accompany your videos with plots that show how you are maintaining the limitations on acceleration and deceleration
  ![Screenshot 2023-11-02 at 20-11-40 Robotics Engineer Assignment - Robotics Engineer Assignment-1 pdf](https://github.com/AshX3301/Ros2-Turtlesim/assets/70807797/743a5487-bf77-45cf-a887-f1052bada31e)

## Goal 2: Solution
- Code: [Goal2 final.py](https://github.com/AshX3301/Ros2-Turtlesim/blob/main/code%20resub/goal%202/goal2%20final.py)
- Video: https://drive.google.com/file/d/1D8Rwd34xeOxKnrymM-4O_CICcAsL7zM7/view?usp=drive_link
