## Goal 3: Rotate turtle in circle
- Write code that will keep moving a turtle from goal 2 in circles. Provide variables to control the speed and radius of the circle.
- Publish the pose of this turtle every 5 secs. Let us call this topic ‘/rt_real_pose’.
- Publish another topic with the pose of the turtle, added with random gaussian noise, every 5 sec. Let us call this topic ‘/rt_noisy_pose’.
- Set the noise standard deviation at 10 units.

## Goal 3: Solution
- Code for continuous pose publication: [goal3final.py](https://github.com/AshX3301/Ros2-Turtlesim/blob/main/code%20resub/Goal%203/goal3final1.py)
- Code for 5 sec interval pose publication: [goal3intercept.py](https://github.com/AshX3301/Ros2-Turtlesim/blob/main/code%20resub/Goal%203/goal3%20intercept.py)

## Results
- Video: https://drive.google.com/file/d/18fFx_LkmLJVQqSmHXkJhEh5NLI8L7hXZ/view?usp=drive_link
- Noisy vs Real Visualization: https://drive.google.com/file/d/1jic7KfTt70xF5wBIHCW4nzzya0bysgSj/view?usp=drive_link
- ![real pose vs noisy pose](https://github.com/AshX3301/Ros2-Turtlesim/assets/70807797/4f2148a2-1746-4a65-a1fd-6eea1c29631b)
