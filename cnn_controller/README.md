# Overview

This package contains a mobile robot controller based on a Convolutional Neural Network (CNN) to takes in lidar data and a goal position and outputs the velocity command for the robot. The package provides action servers for path planning that are compatible with ROS2 Navigation2 (nav2) framework.

# Learning objectives

1. Use supervised learning to train a CNN to control the motion of a mobile robot.
2. Understand the parameters that go into CNN training.
3. Integrate the controller into the robot code architecture.

# Pre-Requisites
None.

# Dependencies

This package requires several additional Python modules:

```bash
pip3 install numpy==1.21.5 scipy==1.10
pip3 install torch==2.4.0 torchvision==0.19.0 torchaudio==2.4.0 --index-url https://download.pytorch.org/whl/cpu
pip3 install tensorboardX
```

This will ensure that you have compatible versions of the different libraries. Note, this installs a CPU version of PyTorch, which should work on any computer. If you have a GPU and want to use that instead, you can replace the second line above with a [different command](https://pytorch.org/get-started/previous-versions/#:~:text=0.19.0%20torchaudio%3D%3D2.4.0-,LINUX%20AND%20WINDOWS,-%23%20ROCM%206.1%20(Linux) (based on the version of the CUDA library that you have installed).

Standard ROS2 packages are also required at runtime (rclpy, tf2_ros, nav2, various messages, etc.).

# File Layout

This directory contains the following files and folders:

- [`package.xml`](package.xml): ROS 2 package manifest with dependencies and metadata.
- [`pytest.ini`](pytest.ini): Pytest configuration file.
- [`setup.cfg`](setup.cfg): Configuration for Python packaging and linting.
- [`setup.py`](setup.py): Python setup script for installing the package.
- [`README.md`](README.md): This file.
- [`resource/cnn_controller`](resource/cnn_controller): Resource file for ROS 2 package index.
- [`test/`](test) - basic package tests (flake8, pep257, copyright)

## [`goal_checker`](goal_checker) Python Package

This package provides a simple checker to determine if the robot's position is close to the desired goal. This is based on the [Goal Checker in `nav2`](https://docs.nav2.org/plugins/index.html#goal-checkers).

- [`goal_checker/__init__.py`](goal_checker/__init__.py): Initializes the `goal_checker` Python package.
- [`goal_checker/goal_checker.py`](goal_checker/goal_checker.py): Implementation (class `GoalChecker`). Checks if the robot is within a given tolerance of the goal position.

## [`cnn_model`](cnn_model) Python Package

This package provides the definition of the CNN model as well as files to train and test your CNN.

- [`cnn_model/__init__.py`](cnn_model/__init__.py): Initializes the `cnn_model` Python package.
- [`cnn_model/cnn_model.py`](cnn_model/cnn_model.py): The main file for this package. Contains helper functions to normalize and unnormalize data, the definition of the `NavDataset` class (which extends PyTorch's `Dataset` class), and the definition of the `CNNModel` class (which defines the CNN model we will use in this project).
- [`cnn_model/turtlebot3_dataset.tar.gz`](cnn_model\turtlebot3_dataset.tar.gz): The dataset you will use to train your model.
- [`cnn_model/cnn_train.py`](cnn_model/cnn_train.py): A Python script that will use PyTorch and the dataset to train the CNN.
- [`cnn_model/cnn_train.sh`](cnn_model/cnn_train.sh): A bash script that calls [`cnn_train.py`](cnn_model/cnn_train.py).
- [`cnn_model/cnn_test.py`](cnn_model/cnn_test.py): A Python script that will use PyTorch and the dataset to test your trained CNN.
- [`cnn_model/cnn_test.sh`](cnn_model/cnn_test.sh): A bash script that calls [`cnn_test.py`](cnn_model/cnn_test.py).

## [`cnn_controller`](cnn_controller) Python Package

This package uses the CNN model inside of a ROS2 node to control the motion of a mobile robot.

- [`cnn_controller/__init__.py`](cnn_controller/__init__.py): Initializes the `cnn_controller` Python package.
- [`cnn_controller/cnn_controller_node.py`](cnn_controller/cnn_controller_node.py): The implementation of the `CNNControllerNode` class. This is a ROS2 Lifecycle Node that uses your trained CNN along with the planned path and lidar scans to generate control commands for the robot.

# Instructions

Your task is to train the CNN and integrate it with ROS2 Navigation2 framework.

## CNN Training

### Unpack the Dataset

The first step in training your CNN is to unpack the dataset. A `.tar.gz` file is a compressed file format (like a `.zip` file). To extract the data, you need to open a terminal, go to this folder, and run the following command:

```bash
tar -xzf turtlebot3_dataset.tar.gz
```

### Tune the Learning Parameters

You need to edit the file [`hyperparameters.yaml`](params/hyperparameters.yaml), which is used during both training *and runtime* in your node. You can change the folling parameters:

1. `goal_input`: There are two available options for which point to use as the input to the CNN. This affects whether you are trying to drive to the final destination or follow along the path.
    * `final_goal`: Use the end point of the path
    * `sub_goal`: Use the point that is a distance `lookahead` (a ROS2 parameter) in front of the robot along the path
2. `num_channels`: The number of parallel data channels used in the CNN. Increasing this will increase the number of parameters in your CNN, which can allow it to learn more complex models but requires additional computation.
    * You can choose any power of two that is $>= 4$
3. `normalization_method`: The method used to normalize the data input into the CNN (as discussed in the ML tutorial assignment).
    * `standardization`: Convert each value to its Z-score (i.e., number of standard deviations above the mean value)
    * `min_max`: Convert each value to a range from [-1, 1]
4. `num_epochs`: The number of training epochs (i.e., the number of times to update the CNN parameters).
    * You can choose any integer $> 0$
5. `batch_size`: The number of samples from the dataset to use in each training batch. Having a small batch will be much faster, while having a larger batch will make each training update step be less reactive to errors in individual samples.
    * You can choose any power of two that is in the range [2, 1024]
6. `loss_function`: The objective function you aim to minimize during training, where the error is the difference between the true control signal and the signal output by your model.
    * `MSE`: The [Mean Square Error](https://docs.pytorch.org/docs/stable/generated/torch.nn.MSELoss.html).
    * `MAE`: The [Mean Absolute Error](https://docs.pytorch.org/docs/stable/generated/torch.nn.modules.loss.L1Loss.html)
7. `learning_rate_exp`: The exponent for the learning rate used in the optimization algorithm, in our case the [Adam optimizer](https://docs.pytorch.org/docs/stable/generated/torch.optim.adam.Adam.html). Here is an [explainer for the Adam optimizer](https://medium.com/@weidagang/demystifying-the-adam-optimizer-in-machine-learning-4401d162cb9e). The learning rate value will be $10^{\rm learning\_rate}$
    * You can choose any float in the range [-6.0, 0.0] (result in values from $10^{-6}$ to 1).
8. `checkpoint_rate`: The number of epochs that pass between saving the model during training.
    * You can choose any integer > 0.
9. `preload_all_data`: Choo whether to pre-load all training data into memory or load on demand. Preloading will increase speed but require more memory usage. It is recommended that you use `True` unless your computer does not have enough RAM.
    * `True` or `False`

### Learning Rate Adjustment
You can adjust the learning rate during training by modifying the [`adjust_learning_rate`](cnn_model/cnn_train.py#L26-L35) function. This function takes in the `base_learning_rate` (parameter 7 from above) and the current training `epoch`. Doing this can be helpful, where the general idea is to take larger steps early on during training, when the model has large error, and then take smaller steps later on, when the model is close to converging. You can make sudden jumps by using `if` statement, use a function to smoothly modify the learning rate as a function of the epoch, etc.

### Train Your Model
To actually train your model you need to open a terminal, go to the [`cnn_model`](cnn_model) folder, and run the [`cnn_train.sh`](cnn_model/cnn_train.sh) script. You can do this with the command

```bash
sh cnn_train.sh
```

You will see a progress bar appear in the terminal window, indicating the percentage completed and giving you two error values, one on the training data and one on the development data. In general, you want both of these errors to be "small," indicating that your model generates outputs that are similar to the actual data. You can interpret the value based on what it represents (a difference between vectors of linear and angular velocities). You also want the errors to be close between the training and development data, as this indicates that your model is not overfitting to the training data.

When the training process finishes, it will copy the final model to [`params/cnn_model_weights.pth`](params). This is where the navigation node expects the file to be.

You can also test on a third collection of data, which was not seen at all during training. To do this, run the command
```bash
sh cnn_test.sh
```
Again, you want the error to be "small" and the value to be close to the error on the training data.

### Navigation Performance
The true test is to make sure your CNN model allows the robot to navigate around safely. There are a few things you need to do to set this up.

1. Make sure you do not change the parameter values in [`hyperparameters.yaml`](params/hyperparameters.yaml) from the time you trained the model to the time you use it. For example, if you change the number of channels the node will error out.

2. Add the `cnn_controller_node` to your launch file in the `<!-- Local controller -->` section. When you do this, make sure you set the argument `use_nav2_controller` in [`simulation.xml`](../mee4411_simulation/launch/simulation.xml#L32-L34) to `False` so that it will use your node (this will also stop the node `controller_server` from running in [`navigation_launch.py`](../mee4411_simulation/launch/navigation_launch.py#L135-L144)). You also need to name your node `controller_server` for it to work properly. Note: your node should only run when `use_nav2_controller` is `False`. You can do this using the [`unless` field in the launch file](https://design.ros2.org/articles/roslaunch_xml.html).

3. I recommend using the new map, [`turtlebot3_world`](../occupancy_grid/maps/turtlebot3_world.yaml). This is an occupancy grid version of the map used in the [Turtlebot3 simulations](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#:~:text=empty_world.launch.py-,TurtleBot3%20World,-%24%20export%20TURTLEBOT3_MODEL).

4. Just like in the global planning project, you can give your robot a goal to drive towards by using the `Goal Pose` tool in `rviz`. The `rviz` configuration has been updated to include markers that show the goal your robot is driving towards as well as a circle around the robot to indicate the zone within which it considers having reached the goal (i.e., it will stop when the final goes is inside this zone).

5. Test path planning with Navigation2:

```bash
ros2 launch mee4411_simulation simulation.xml
```
