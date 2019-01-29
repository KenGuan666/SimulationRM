# Robomaster-AI-Challenge-2019

## Installation

1. Install OpenAI Gym with `pip install gym`, OpenCV with `pip install opencv-python` and keyboard control with `pip install keyboard`.


2. Clone the repo anywhere, then copy the entire `/DJI` folder into `<python-packages-root-directory>/gym/env/`.

3. In `<python-packages-root-directory>/gym/env/__init__.py`, add the following code parallel to similar clauses:


`register(
    id='Robomaster-v0',
    entry_point='gym.envs.DJI:RobomasterEnv',
    max_episode_steps=9999,
    reward_threshold=90.0,
)`

## Running

To run the simulation, execute the python script `.../DJI/robot_play.py`.

Updated 1/29/2019
