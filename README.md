**UAV Strategic Deconfliction in Shared Airspace System**


A modular Python toolkit for simulating, detecting, and resolving 3D airspace conflicts between multiple drones. It generates animated GIFs showing both conflict points and the delay-resolved trajectories.


**Features:**
Waypoint-based drone missions in 3D space

Linear path interpolation over mission time windows

Conflict detection with configurable safety buffer

Automated conflict resolution via minimal delay

3D animation (GIF) of both conflict and resolved scenarios

Detailed terminal logs of conflict intervals and positions


⚙️**Requirements**
Python 3.7+
NumPy
SciPy
Matplotlib
(Optional) ImageMagick or Pillow for GIF encoding


Install dependencies via:
pip install numpy scipy matplotlib pillow


📁**Project Structure**
.
├── models.py             # Waypoint & Drone data classes

├── conflict_detector.py  # Path interpolation, conflict detection, and resolution

├── simulator.py          # 3D visualization & animation

├── main.py               # Example entry point (uses `specs` to instantiate drones)

└── README.md             # Project overview and usage


🔧 Installation
Clone this repository:
git clone https://github.com/happiesthd/flytbase-assignment-harshada.git
cd flytbase-assignment-harshada


▶️**Usage**
Edit main.py to define your specs list:

specs = [
  ("Drone1", 0, 50, [(0,0,0),(25,25,25),(50,50,50),(75,75,75),(100,100,100)]),
 #other drones specify T_start, T_end and waypoint coordinate(x,y,z)
]


**Run:**
python main.py


**Output:**

mission_conflict.gif – shows original conflicts

mission_resolved.gif – shows delayed (resolved) path with original conflict markers(in red)




