# RocketSim Utils

Based off of [RocketSim v3](https://github.com/ZealanL/RocketSim/tree/v3-rust),
RocketSim utils is a collection of various single-purpose tools that are at least 2x faster than using a full RocketSim instance.

## ball_sim

Simulates 1 ball, no cars, no boost pads. Requires collision meshes.
6 seconds of simulation (720 ticks) takes about 38 microseconds on average on a 5900X, which is over 19 million TPS (in-game Ticks Per real-life Second).

## ball_sim_cpp

C++ bindings for `ball_sim` built with the CXX crate. Includes a CMake example project under `ball_sim_cpp/make_example` that builds the Rust static library via Cargo and links a C++ executable against it.

## drive_sim

Simulates 1 car driving on the floor with boost pads, no ball. Does not require collision meshes.
6 seconds of simulation (720 ticks) takes about 90 microseconds on average on a 5900X, which is about 8 million TPS (in-game Ticks Per real-life Second).

## air_sim

Simulates 1 car in the air, no ball, no boost pads. Does not require collision meshes.
6 seconds of simulation (720 ticks) takes about 57 microseconds on average on a 5900X, which is over 12 million TPS (in-game Ticks Per real-life Second).

## turn_sim

Simulates 1 car turning while suspended in the air. Does not require collision meshes.
6 seconds of simulation (720 ticks) takes about 37 microseconds on average on a 5900X, which is about 19 million TPS (in-game Ticks Per real-life Second).
