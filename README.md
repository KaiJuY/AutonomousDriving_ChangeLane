# AutonomousDriving_ChangeLane
using MATLAB MPC tool box to predictive collusion will happend then controller calculation correct steering angle for ego-vehicle cross over another vehicle .

# Simulation figure
in this algorithm only develop on simple situation.
1. Driver in safe path :

![Match_Trace](https://github.com/KaiJuY/AutonomousDriving_ChangeLane/assets/138283005/2911cb28-9d93-403e-9ea6-cb0e28b28df1)

Controller will folling driver control to driver vehicle.

2. Driver in unsafe path :

![Compare_Mix_Trace](https://github.com/KaiJuY/AutonomousDriving_ChangeLane/assets/138283005/aee31354-be29-49af-a6a3-5b420901984f)

Controller will avoid collusion the black line is MPC controller path, the green line is driver path.

3. Finally Vehicle path:

![Mix_Trace](https://github.com/KaiJuY/AutonomousDriving_ChangeLane/assets/138283005/ea31cd79-0bd7-467e-a5c7-eda9c87de0df)

this result inclue safe and unsafe situation begin ego-vehicle is safety MPC controller will fowlling driver,
then MPC predictive risk for collusion controller will avoid this and control the ego-vehicle.

Here has the report for this project

[AutonomousDriving_ChangeLane.pdf](https://github.com/KaiJuY/AutonomousDriving_ChangeLane/files/11925050/AutonomousDriving_ChangeLane.pdf)
