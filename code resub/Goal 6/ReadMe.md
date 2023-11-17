## Goal 6: Chase turtle noisy
- Same setup as Goal 5:
- Only difference being, the PT can access RT’s noisy pose every 5 secs through topic ‘rt_noisy_pose’.
- PT needs to chase down RT.
- Do you need a better estimator to guess where the RT is going to be?


## Goal 6: Solution
- Code for continuous data: [Goal6.py](https://github.com/AshX3301/Ros2-Turtlesim/blob/main/code%20resub/Goal%206/goal6.py)
- Code for 5 sec interval data: [goal6 kalman intercept.py](https://github.com/AshX3301/Ros2-Turtlesim/blob/main/code%20resub/Goal%206/goal6_kalman_intercept.py)

## Result
- Video: https://drive.google.com/file/d/1Ejq27FOukSf75fMU_p4V8A1rAmxPesdT/view?usp=drive_link
- Video: https://drive.google.com/file/d/1TrkcRdo8PaKcgnLthE_7auX14gHc0w6E/view?usp=drive_link
- Video: https://drive.google.com/file/d/1RdkukrtEsY_9Nd7lQeKQ5j2UEuFeb9ou/view?usp=drive_link

- Without kalman filter path

![Goal 6 ](https://github.com/AshX3301/Ros2-Turtlesim/assets/70807797/75d7bd4f-c6f4-4664-91e0-35a72e97a789)

- Kalman Filter Performance

![goal6 kalman filter pose data](https://github.com/AshX3301/Ros2-Turtlesim/assets/70807797/4b48e4a6-e421-45aa-9864-a6533cfa2dd1)
