# v3.6.1
The z1_sdk is mainly used for communication between z1_controller and command panel(keyboard, joystick, user control panel etc).

## Notice

support robot: Z1(v3.6.1)
not support robot: Z1(v3.4, v3.5)

## Dependencies

- build-essential

```bash
sudo apt install build-essential
```

- Boost (version 1.5.4 or higher)

```bash
dpkg -S /usr/include/boost/version.hpp	# check boost version
sudo apt install libboost-dev			# install boost
```

- CMake (version 2.8.3 or higher)

```bash
cmake --version 		# check cmake version
sudo apt install cmake	# install cmake
```

- [Eigen](https://gitlab.com/libeigen/eigen/-/releases/3.3.9) (version 3.3.9 or higher)

```bash
cd eigen-3.3.9
mkdir build && cd build
cmake ..
sudo make install
sudo ln -s /usr/local/include/eigen3  /usr/include/eigen3
sudo ln -s /usr/local/include/eigen3/Eigen  /usr/local/include/Eigen
```

## Example

There are three examples of command panel control based on SDK in this project, you can use it fllowing the steps below.

### State change

- First, set(CTRL_PANEL SDK)  # z1_ws/src/z1_controller/CMakeList.txt，and then rebuild the z1_ws to generate z1_ctrl, then open a teminal to run z1_ctrl

  ```
  cd /<path to>/z1_controller/build
  sudo ./z1_ctrl
  ```

- Sencond, build the z1_sdk, and then open another terminal to run example.

  ```
  cd /<path to>/z1_sdk && mkdir build && cd build
  cmake ..
  make -j4
  ./example_state_send
  ```

### Low level control

```
sudo ./z1_ctrl			# Running in a terminal
./example_lowCmd_send	# Running in another terminal
```

### Keyboard control

```
sudo ./z1_ctrl				# Running in a terminal
./example_keyboard_send		# Running in another terminal
```
