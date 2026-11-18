"""
simulink_connector.py
Connect and interact with MATLAB Simulink from Python using the MATLAB Engine API
for the Custom AGV 3D Autonomous Navigation simulation.
"""

import matlab.engine
from typing import Optional, Any, Dict


class SimulinkBridge:
    def __init__(self, shared_session_name: Optional[str] = None):
        """
        Initialize the connection to MATLAB.
        """
        sessions = matlab.engine.find_matlab()
        if shared_session_name:
            print(f"Connecting to shared MATLAB session '{shared_session_name}'...")
            self.eng = matlab.engine.connect_matlab(shared_session_name)
        elif len(sessions) > 0:
            print(f"Connecting to existing shared MATLAB session '{sessions[0]}'...")
            self.eng = matlab.engine.connect_matlab(sessions[0])
        else:
            print("Starting a new MATLAB engine instance (this may take a few seconds)...")
            self.eng = matlab.engine.start_matlab()

        print("Connected to MATLAB successfully!")

    def open_simulink_library(self):
        """Open the Simulink Start Page / Library browser."""
        self.eng.simulink(nargout=0)

    def load_model(self, model_name: str = 'agv_3d_nav'):
        """Load a Simulink model (.slx) into memory."""
        print(f"Loading Simulink model '{model_name}'...")
        self.eng.load_system(model_name, nargout=0)

    def open_model(self, model_name: str = 'agv_3d_nav'):
        """Open the Simulink model inside the MATLAB graphical editor."""
        print(f"Opening Simulink model '{model_name}' in editor...")
        self.eng.open_system(model_name, nargout=0)

    def set_workspace_variable(self, var_name: str, value: Any):
        """Set a variable in the MATLAB base workspace for the model to use."""
        self.eng.workspace[var_name] = value

    def get_workspace_variable(self, var_name: str) -> Any:
        """Retrieve a variable from the MATLAB base workspace."""
        return self.eng.workspace[var_name]

    def run_simulation(self, model_name: str = 'agv_3d_nav', stop_time: Optional[float] = None) -> Any:
        """
        Run the Simulink / Simscape Multibody simulation.
        Mechanics Explorer will open automatically and animate the 3D physics.
        """
        if stop_time is not None:
            self.eng.set_param(model_name, 'StopTime', str(stop_time), nargout=0)

        print(f"Running 3D Simscape simulation '{model_name}' in Mechanics Explorer...")
        sim_out = self.eng.sim(model_name)
        print("Simulation completed successfully!")
        return sim_out

    def set_navigation_target(self, target_x: float, target_y: float, target_yaw: float = 1.5708):
        """Replan Hybrid A* trajectory and update 3D target beacons."""
        print(f"Re-planning 3D navigation path to Target: [{target_x}, {target_y}]...")
        self.eng.build_agv_3d_simulation(float(target_x), float(target_y), float(target_yaw), nargout=0)
        print("Trajectory and 3D environment updated!")

    def close_model(self, model_name: str = 'agv_3d_nav', save: bool = False):
        """Close the model."""
        save_val = 1 if save else 0
        self.eng.close_system(model_name, save_val, nargout=0)
        print(f"Closed '{model_name}'.")

    def disconnect(self):
        """Close connection to MATLAB Engine."""
        self.eng.quit()
        print("Disconnected from MATLAB Engine.")
