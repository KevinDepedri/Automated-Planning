# <p align="center">Automated Planning - Planning Assignment</p> 

This repository is a student project developed by Kevin Depedri for the "Automated Planning" course of the Master in Artificial Intelligent Systems at the University of Trento, a.y. 2022-2023.

The repository is composed of:
- Assignment pdf
- A folder for each one of the 5 task described in the assignment
- Report pdf
- Guide to install all the needed material to run the code (this readme)

Description of the 5 different tasks:
  - Task 1: Basic structure of the problem developed in PDDL. Here we have a robotic agent which is appoited to bring the required supplies to some injured people. To run this task FastDownard planner is required.
  - Task 2: Extension of the previous problem developed in PDDL. Here the robotic agent has at its disposal a carrier which allows to transport supplies in a more efficient way. To run this task FastDownard planner is required.
  - Task 3: Conversion of task 2 from PDDL to HDDL. Here tasks and methods are introduced and made compatible with the previously developed actions. To run this task Panda planner is required.
  - Task 4: Extension of task 2 to temportal domain. All the actions have been updated with a specific duration and with some time constraints. The goal is obtained minimizing the required time. To run this task Optic planner or TemporalFastDownward planner is required.
  - Task 5: Extension of task 4 to a more sophisticate planner which allows to define a C++ code for each action. Once a plan is obtained (minimizing the time required) it is possible to run the plan, simulating the behavior of the proposed solution. The simulation is based on the implemented C++ codes. To run this task ROS2 and PlanSys2 are required.

All the previously listed planners used are available and ready to install on Linux. The best option is to use a Linux machine. Another good and quick option is to install the Linux Kernel directly in Windows (supported on Windows 10/11). 
To install the Linux kernel on Windows follow the next section. Otherwise, go directly to the section regarding the installation of the planners on Linux.

****
# Installing WSL and Ubuntu
WSL(Windows Subsystem for Linux) is the main component required to install and run a Linux Kernel directly on Windows. Once WSL is inistalled it is possible to install the Linux distribution that you prefer. Follow the ensuing steps to enable WSL and install the Ubuntu distro.
  1. Enable virtualizzation (task manager -> cpu -> virtualization, to see if it is already enabled). If it is not then enable it from the BIOS
  2. From start search for 'Enable disable windows features', here make sure 'Windows Subsystem for Linux' is enabled
  3. Run in powershell
  ```shell
  wsl --install
  ```
  4. Download [Ubuntu from Windows store](https://www.microsoft.com/store/productId/9PDXGNCFSCZV)
  5. Run Ubuntu directly from the installed icon or from Windows terminal (best option to use multiple terminals)
  6. If Windows terminal is not installed then [download it from the Windows store](https://www.microsoft.com/store/productId/9N0DX20HK701)

Once in your Ubuntu run the following line of code to update your package and to get ready for the next steps
```bash
sudo apt update
sudo apt upgrade
sudo apt-get python3-pip
```

****
# Installing the planners
## PLANUTILS
Planutils is a suit of planners that allows to install a virtual environment where planners can be quickly installed and runned. It requires singularity to work, which is based on GO. Follow the ensuing steps to install it.
1. Install GO and Singularity
```bash
sudo apt-get update && \
sudo apt-get install -y build-essential \
libseccomp-dev pkg-config squashfs-tools cryptsetup

sudo rm -r /usr/local/go

export VERSION=1.17 OS=linux ARCH=amd64  # change this as you need

wget -O /tmp/go${VERSION}.${OS}-${ARCH}.tar.gz https://dl.google.com/go/go${VERSION}.${OS}-${ARCH}.tar.gz && \
sudo tar -C /usr/local -xzf /tmp/go${VERSION}.${OS}-${ARCH}.tar.gz

echo 'export GOPATH=${HOME}/go' >> ~/.bashrc && \
echo 'export PATH=/usr/local/go/bin:${PATH}:${GOPATH}/bin' >> ~/.bashrc && \
source ~/.bashrc

curl -sfL https://install.goreleaser.com/github.com/golangci/golangci-lint.sh |
sh -s -- -b $(go env GOPATH)/bin v1.21.0

mkdir -p ${GOPATH}/src/github.com/sylabs && \
cd ${GOPATH}/src/github.com/sylabs && \
git clone https://github.com/sylabs/singularity.git && \
cd singularity

git checkout v3.6.3

cd ${GOPATH}/src/github.com/sylabs/singularity && \
./mconfig && \
cd ./builddir && \
make && \
sudo make install
```
  
2. Fix singularity mount host options
```bash
cd /usr/local/etc/singularity
vim singularity.conf
# Now press 'shift+I' to enter insert mode
# Navigate down the file until the voice 'mount hostfs = no' is found
# Change 'mount hostfs = no' in 'mount hostfs = yes'
# Press 'esc', then type ':x' and press enter to save and exit
```

3. Install Planutils
```bash
pip install planutils
```
4. Activate and setup planutils 
```bash
activate planutils
planutils setup
```
5. Install the required planners
```bash
planutils install <downward|panda|tfd|optic>  #Install one planner at time
```

## PLANSYS2
PlanSys2 is based on ROS2. Furthermore, 2 more packages are required to build the dependencies of the project (Rosdep) and to compile it (Colcon for ROS). Follow the ensuing steps to install everything

### ROS2
1. Set locale
  ```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
  ```
  
2. Setup sources
  ```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
  ```

3. Add the ROS 2 GPG key with apt
  ```bash
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  ```

4. Add the repository to your sources list
  ```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  ```

5. Update your apt repository caches after setting up the repositories and verify that all the currently installed packages are up to date before installing ROS2
  ```bash
sudo apt update
sudo apt upgrade
  ```

6. Install ROS-Base (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools (useless since we are working from WSL terminal).
  ```bash
sudo apt install ros-humble-ros-base
  ```

### COLCON for ROS2
1. Get the files from the ROS2 repository and add its key to apt
  ```bash
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  ```

2. Update your apt repository caches after setting up the repositories and the install the package using apt
  ```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
  ```

### ROSDEP
1. Install rosdep
  ```bash
sudo apt-get install python3-rosdep
  ```

2. Initialize and update it
  ```bash
sudo rosdep init
rosdep update
  ```

***
# Running the planners
To run the planners on the domain and problem files of this assignment, first of all download this repository
1. Get to your workspace
  ```bash
cd <your_workspace>
  ```
  
2. Get the github repo and store it in the `planning` folder
  ```bash
git clone https://github.com/KevinDepedri/Automated-Planning.git planning
  ```
  
## PLANUTILS
To run one of the planner installed in planutils follow the ensuing procedure
1. Get to the folder of one of the following task
  ```bash
cd <your_workspace>/planning/{task1|task2|task3|task4}
  ```

2. Activate planutils 
```bash
activate planutils
```

3. Run the correct planner for that task according to the following list:
- Task1 & Task 2: downward
- Task3: panda
- Task4: optic or tfd (temporal fast downward)
```bash
<downward|panda|tfd|optic> your_domain.pddl your_problem.pddl  #Or .hddl for Panda
```

## PLANSYS2 
Two terminals are necessary to run PlanSys2.
#### TERMINAL 1
Terminal one is used to build the dependencies, compile the project and host the PlanSys2 planner based on ROS. To do so follow the ensuing procedure
1. Get inside the project folder
  ```bash
cd <your_workspace>/planning/task5/plansys2_task5/
  ```

2. Install ROS2 infrastrucutre for the current terminal
  ```bash
source /opt/ros/humble/setup.bash
  ```

3. Compile the project a first time, it could lead to errors
  ```bash
colcon build --symlink-install
  ```

4. Now, install all the dependencies required by that project
  ```bash
rosdep install --from-paths ./ --ignore-src -r -y
  ```

5. Compile the project again, this time the compilation will be sucessful. If problem arises try to compile the code multiple times
  ```bash 
colcon build --symlink-install
  ```

6. Integrate ROS2 infrastrucutre with the PlanSys2 compiled in the previous step
  ```bash
source install/setup.bash
  ```

7. Luch ROS2. Now this terminal will be used only to show the results
  ```bash
ros2 launch plansys2_task5 plansys2_task5_launch.py
  ```
  
All the above listed procedure for terminal one is automatically exetcuted by running the file `compile_and_run.sh` present in the scripts folder. If you want to use it remember first to open it and to change the path in the first line with the correct path for your system.

#### TERINAL 2
Once terminal 1 has been set up, open a new terminal. Terminal two is used to run the PlanSys2_terminal, which is used to push into the planner all the wanted data (instances, predicates, goal), to compute a plan and to run it.

1. Repeat step 1, 2 and 6 also for this terminal
  ```bash
cd <your_workspace>/planning/task5/plansys2_task5/
surce /opt/ros/humble/setup.bash
source install/setup.bash
  ```

2. Run ROS2-terminal
  ```bash
ros2 run plansys2_terminal plansys2_terminal
  ```

3. Source the problem data (using the absolute path from your system root)
  ```bash
source /mnt/c/Users/you_user/your_workspace/downloaded/plansys2_task/launch/problem_data 1  #Update this path according to your system
  ```

4. Get a plan
  ```bash
get plan
  ```

5. Run the plan. The status is visualized on the this terminal, the sum-up is visualized on the first terminal
  ```bash
run
  ```
  
The first two steps of the above listed procedure for terminal two are automatically exetcuted by running the file `run_terminal.sh` present in the scripts folder. If you want to use it remember first to open it and to change the path in the first line with the correct path for your system.

***
# Results obtained running the planners
## Task 1
Running task 1 using *Downward* from planutils returns the following plan

![Task-1: Downward run with alias lama, solution 1](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%201/Lama%20-%20Solution%201.PNG)

The search is then terminated since no better plan can be found. The plan found above is optimal

![Task-1: Downward run with alias lama, end of search](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%201/Lama%20-%20Solution%202%20-%20End%20of%20search.PNG)

## Task 2
Running task 2 using *Downward* from planutils returns the following plan

![Task-2: Downward run with alias lama, solution 1](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%202/Lama%20-%20Solution%201.PNG)

After a few second, the search returns the following improved plan

![Task-2: Downward run with alias lama, solution 2](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%202/Lama%20-%20Solution%202.PNG)

After almost 15 minutes, the searchs returns the following improved plan

![Task-2: Downward run with alias lama, solution 3](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%202/Lama%20-%20Solution%203.PNG)

The search is then keept on running for more than 2 hours and eventually crashed due to memory constrains. Anyway, looking at the last plan computed we can assess that it is the optimal one, and that no better plan could be found

![Task-2: Downward run with alias lama, end of search](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%202/Lama%20-%20Solution%204%20-%20Crash%20at%202GB%20of%20RAM%20after%202hours.PNG)

## Task 3
Running task 3 using *Panda* from planutils returns the following plan

![Task-3: Panda run, solution 1](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%203/PANDA%20-%20suboptimal%20-%20Solution%20with%20all%20methods%20active.PNG)

That solution is based on the following list of hierarchical tasks, methods and actions
![Task-3: Panda run, solution 1, hierarchical](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%203/PANDA%20-%20suboptimal%20-%20Solution%20with%20all%20methods%20active%20-%20hierarchy.PNG)

We can notice that this plan is not optimal due to the task `deliver_by_robot`. For this reason, since no options are available on panda to perform a larger search and look for alternative solutions, we have decided to comment the task that led to this suboptimal behavior. Then we ran again the search obtaining the following plan

![Task-3: Panda run, solution 2](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%203/PANDA%20-%20optimal%20-%20Solution%20commenting%20deliver_by_robot.PNG)

That solution is based on the following list of hierarchical tasks, methods and actions

![Task-3: Panda run, solution 2, hierarchical](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%203/PANDA%20-%20optimal%20-%20Solution%20commenting%20deliver_by_robot%20-%20hierarchy.PNG)

We can notice how this plan results to be optimal

## Task 4
Running task 4 using *Optic* from planutils returns the following plan

![Task-4: Optic run, solution 1](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%204/Optic%20-%20Solution%201%20-%20suboptimal.PNG)

The search is then terminated since Optic does not allow to look for other plans. Unfortunately, the plan found above is suboptimal, this is due to the fact that the robot unload and loads itself uselessly at second 19 and 20, loosing 2 second over the time of the possible optimal solution.

## Task 5
Running task 5 using PlanSys2 returns the following plan on the PlanSys2 terminal

![Task-5: PlanSys2, solution 1](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%205/ROS2%20-%20Solution%201%20-%20Suboptimal.PNG)

In the meanwhile, the PlanSys2 planner on the first terminal returns the following sum up of the actions performed
![Task-5: PlanSys2, solution 1](https://github.com/KevinDepedri/Automated-Planning/blob/main/computed_plans/Task%205/ROS2%20-%20Solution%201%20-%20Suboptimal%20-%20RUN.PNG)

The search is then terminated since PlanSys2 does not allow to look for other plans. Unfortunately, the plan found above is suboptimal, this is due to the fact that the robot unload and loads itself uselessly at second 23 and 28, loosing 2 second over the time of the possible optimal solution.
