# SocNavBench Baseline Algorithms

## Welcome to a small repository of baseline algorithm implementations for SocNavBench

This collection contains the implementation for the following robot planner algorithms interfaced through the JoystickAPI to the SocNavBench simulator. 
- [Random](joystick_py/joystick_random.py) (naive) (`python3`)
- [RandomCpp](joystick_cpp/) (naive) (`c++`)
- [Sampling](joystick_py/joystick_planner.py) (naive) (`python3`)
- [RVO](RVO2/) (`c++`)
- [RVOwCkpt](RVO2/) (`c++`)
- [Social forces](social_force/) (`c++`)

**The following require installing the implementation from this [repo](https://github.com/mit-acl/gym-collision-avoidance) (see below in *SACADRL implementation*)**
- [SACADRL](sacadrl/) (`python3`)
- [SACADRLwCkpt](sacadrl/) (`python3`)

## Building instructions
### TODO: see todo #2
All c++ implementations **should** have valid makefiles where you can just run `make`. This has been tested on Ubuntu 21.04.

**NOTE** For running the `social_force/social_force` executable, you'll first need to run
```bash
cd /path/to/SocNavBench/joystick/social_force/
export LD_LIBRARY_PATH=./src
# then you can safely run
./social_force
```
This is not the case for the `RVO2` binary which can simply be executed with
```bash
cd /path/to/SocNavBench/joystick/RVO2/
./RVO2
```

**NOTE** these binaries require executable permissions, so after compilation you'll need to give these with `chmod a+x`.

## SocNavBench integration
first off, you'll need to copy the contents of this entire repo and replace the current `joystick/` implementation in `SocNavBench`
```bash
cd /path/to/SocNavBench-baselines/
rm -rf /path/to/SocNavBench/joystick/* # clear old directory
cp -r * /path/to/SocNavBench/joystick/ # copy all new files over
```

## Running instructions
**IMPORTANT** make sure you already integrated this codebase with SocNavBench as described in **SocNavBench integration**

For all except the randomCpp joystick implementations, you'll be able to run the python joystickAPI interface through the `joystick_client.py` file. Eg.
```bash
cd /path/to/SocNavBench/
PYTHONPATH='.' python3 joystick/joystick_client.py --algo sampling # sampling planner
```
You may also be interested in passing specific algorithms to run with the `--algo` flag. See the [argparser details](joystick_client.py) for all available options. 

**Note** that most of the `c++` joystick implementations interface through `python` (the exception being `joystick_client.cpp` (random c++ joystick)) but will still have separate executables from their respective source code. 
- Therefore in order to run the RVO, RVOwCkpt, or SocialForces planners, you'll need three terminals to run:
1. the `SocNavBench` simulator itself
2. the `joystick_client.py` client
3. the c++ binary ([`RVO2`](RVO2/RVO2) or [`social_force`](social_force/social_force))

## SACADRL implementation
The implementation for SACADRL uses [this source code](https://github.com/mit-acl/gym-collision-avoidance) and can be installed as follows:
- Install the dependencies mentioned [here](https://github.com/mit-acl/gym-collision-avoidance/blob/f612a44a4511b9fbeb9d6219ba81b7a70a70e793/setup.py#L8-L21)
- Clone the repo and copy the `gym_collision_avoidance/` directory to `joystick/` directly (NOT `joystick/sacadrl`)
- Then you should be able to run the `joystick_client.py` script with `--algo sacadrl` and `--also sacadrlwckpt` without issue. (Make sure to keep the `PYTHONPATH` set to the socnav base dir)


## TODOs:
- Add unit tests
- Refactor c++ implementations and building instructions
- Add full support for both modes of robot motion (system dynamics or positional dynamics)
    - currently positional dynamics is 100% supported, but system (velocity control) dynamics is still in development