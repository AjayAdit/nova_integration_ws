# NOVA integration test

## 1. Fly with windows

```bash
cd "$HOME/ACP DR/nova_integration_ws"
source "$HOME/anaconda3/etc/profile.d/conda.sh"
conda activate All_Drones
source /opt/ros/humble/setup.bash
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$HOME/acados/lib:$LD_LIBRARY_PATH
[ -f install/setup.bash ] || ./setup_ws.sh
source install/setup.bash
ros2 launch launch/nova.launch.py duration:=20.0 viewer:=true detection_view:=true
```

## 2. Plot the flight

```bash
"$HOME/ACP DR/nova_integration_ws/plot_latest.sh"
```
