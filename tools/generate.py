import argparse
import time
from pathlib import Path

from carla_data_generator.simulator1 import CarlaSimulator


def main():
    parser = argparse.ArgumentParser()

    root_path = Path("data")
    velodyne_path = root_path / "velodyne"
    labels_path = root_path / "labels"
    if not velodyne_path.exists():
        velodyne_path.mkdir(parents=True)
    if not labels_path.exists():
        labels_path.mkdir(parents=True)

    simulator = CarlaSimulator()
    simulator.setup_vehicles()
    time.sleep(3)
    simulator.setup_lidars()

    while True:
        simulator.tick()


if __name__ == "__main__":
    main()
