## Goal 4: Chase turtle fast
- Use the turtle from Goal 3 (which can move into circles), let us call it RT (Robber Turtle).
- Use the turtle with limits on acceleration and deceleration, let us call it PT (Police Turtle).
- Spawn RT from location A, which will then keep moving in circles. Set circle radius to more than 30 units.
- Spawn PT from a random location, 10 secs after RT is launched.
- The PT can access RT’s real position every 5 secs through topic ‘rt_real_pose’.
- The PT can move at a higher speed than RT.
- PT needs to chase down RT. Whenever the distance between both of them is less than or equal to a 3 unit distance, we can say that the chase is complete


## Goal 4: Solution
- Code for continuous data: [goal4 final.py](https://github.com/AshX3301/Ros2-Turtlesim/blob/main/code%20resub/goal%204/goal4final.py)
- Code for 5 sec interval data: [goal4 intercept.py](https://github.com/AshX3301/Ros2-Turtlesim/blob/main/code%20resub/goal%204/goal4%20intercept.py)

## Results
- Video: https://drive.google.com/file/d/1doQiBu-EWyol1_vHc49d1Ijjzt9kNQ65/view?usp=drive_link
- Video: https://drive.google.com/file/d/1wWGX2aiLPTF3bLC6jsY8JUT6OyI7QsJ-/view?usp=drive_link
