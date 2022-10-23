import argparse
import time
from pathlib import Path

from carla_data_generator.simulator1_yaml import CarlaSimulator


def main():
    parser = argparse.ArgumentParser()

    root_path = Path("data/carla_raw")
    if not root_path.exists():
        root_path.mkdir(parents=True)

    simulator = CarlaSimulator()
    simulator.setup_vehicles()
    time.sleep(3)
    simulator.setup_lidars()

    while True:
        simulator.tick()


if __name__ == "__main__":
    main()
