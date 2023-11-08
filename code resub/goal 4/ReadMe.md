## Goal 4: Chase turtle fast
- Use the turtle from Goal 3 (which can move into circles), let us call it RT (Robber Turtle).
- Use the turtle with limits on acceleration and deceleration, let us call it PT (Police Turtle).
- Spawn RT from location A, which will then keep moving in circles. Set circle radius to more than 30 units.
- Spawn PT from a random location, 10 secs after RT is launched.
- The PT can access RT’s real position every 5 secs through topic ‘rt_real_pose’.
- The PT can move at a higher speed than RT.
- PT needs to chase down RT. Whenever the distance between both of them is less than or equal to a 3 unit distance, we can say that the chase is complete
