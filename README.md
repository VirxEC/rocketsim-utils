# RocketSim Utils

Based off of [RocketSim v3](https://github.com/ZealanL/RocketSim/tree/v3-rust),
RocketSim utils is a collection of various single-purpose tools that are at least 2x faster than using a full RocketSim instance.

## ball_sim

Simulates 1 ball, no cars, no boost pads. Requires collision meshes.
6 seconds of simulation (720 ticks) takes about 25 microseconds on average on a 5900X, which is over 28 million TPS (in-game Ticks Per real-life Second).

## car_sim

Simulates 1 car driving on the floor with boost pads, no ball. Does not require collision meshes.
6 seconds of simulation (720 ticks) takes about 180 microseconds on average on a 5900X, which is over 3.5 million TPS (in-game Ticks Per real-life Second).
