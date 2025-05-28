import os
import subprocess
import rospy

class PATHelper:
    def __init__(self, robot_id):
        self.robot_id = robot_id

        home = os.environ.get("HOME")
        catkin_root = os.environ.get("CATKIN_ROOT_DIRECTORY")
        pat_root = os.environ.get("PAT_ROOT_DIRECTORY", f"{home}/PAT")
        src = f"{catkin_root}/src/my_sim_pkg/pat"

        self.pizza_output = f"{src}/robot{robot_id}/pizzaDeliveryOutput.txt"
        self.go_home_output = f"{src}/robot{robot_id}/goHomeOutput.txt"
        self.deliver_csp = f"{src}/robot{robot_id}/robot{robot_id}deliverpizza.csp"
        self.gohome_csp = f"{src}/robot{robot_id}/robot{robot_id}gohome.csp"
        self.pat_exec = f"{pat_root}/PAT3.Console.exe"
        self.bfs_timeout = 10  # seconds

    def get_path(self, goal):
        if goal == "delivering":
            cmd = f"timeout {self.bfs_timeout}s mono {self.pat_exec} -engine 1 {self.deliver_csp} {self.pizza_output}"
            output_file = self.pizza_output
        elif goal == "home":
            cmd = f"mono {self.pat_exec} {self.gohome_csp} {self.go_home_output}"
            output_file = self.go_home_output
        else:
            raise ValueError("Invalid goal")

        subprocess.run(cmd, shell=True)
        path = []

        with open(output_file, "r") as f:
            for line in f:
                if line.startswith("<"):
                    moves = line.split("->")[1:]
                    for m in moves:
                        move = m.strip().split()[0]
                        path.append(move.replace(">", "").replace("r", ""))  # Clean up suffix
                    break

        return path
