from concurrent.futures import ThreadPoolExecutor
from functools import cache
from queue import Empty, Queue
from threading import Event
from time import sleep
from traceback import print_exc
#
from cffi import FFI
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation


NUM_ITERATIONS = 100


class ControlSystem:
    def __init__(self):
        self.lib = self.load_native_module()
        self.obj = self.lib.ControlSystem_new()

    def run(self, num_iterations: int):
        self.lib.ControlSystem_run(self.obj, num_iterations)

    def get_temperature(self) -> int:
        return self.lib.ControlSystem_get_temperature(self.obj)

    def get_pressure(self) -> int:
        return self.lib.ControlSystem_get_pressure(self.obj)

    @staticmethod
    @cache  # run only once
    def load_native_module():
        ffi = FFI()
        # Load the shared library
        lib = ffi.dlopen("./control_system.dll")
        # Define the C functions that we want to use
        ffi.cdef("""
                void* ControlSystem_new();
                void ControlSystem_run(void*, int);
                int ControlSystem_get_temperature(void*);
                int ControlSystem_get_pressure(void*);
                """)
        return lib


def get_sensor_data(cs: ControlSystem, stop_event: Event, queue: Queue):
    while not stop_event.is_set():
        temperature = cs.get_temperature()
        pressure = cs.get_pressure()
        print(type(temperature), type(pressure))
        queue.put((temperature, pressure))
        sleep(0.1)


def do_plot(queue: Queue):
    fig, ax = plt.subplots()
    # Initialize the data and line objects
    x_data = []
    temp_data = []
    pressure_data = []
    (temp_line,) = ax.plot(x_data, temp_data, label="Temperature")
    (pressure_line,) = ax.plot(x_data, pressure_data, label="Pressure")
    # Add horizontal lines for the setpoints
    ax.axhline(
        y=20, color="r", linestyle="--", label="Temperature Setpoint"
    )
    ax.axhline(
        y=1, color="g", linestyle="--", label="Pressure Setpoint"
    )
    def update_plot(num, x_data: list, temp_data: list, pressure_data: list, temp_line, pressure_line):
        try:
            temperature, pressure = queue.get_nowait()
            x_data.append(num)
            temp_data.append(temperature)
            pressure_data.append(pressure)
            temp_line.set_data(x_data, temp_data)
            pressure_line.set_data(x_data, pressure_data)
            ax.relim()
            ax.autoscale_view()
            return temp_line, pressure_line
        except Empty:
            return temp_line, pressure_line

    _ = FuncAnimation(
        fig,
        update_plot,
        frames=range(NUM_ITERATIONS),
        fargs=(x_data, temp_data, pressure_data, temp_line, pressure_line),
    )
    ax.legend()
    plt.show()


def main():
    print("Starting...")
    cs = ControlSystem()
    stop_event = Event()
    sensor_data = Queue()
    with ThreadPoolExecutor(max_workers=10) as exec:
        print("Starting backend")
        futures = [exec.submit(f) for f in [
            (lambda: cs.run(NUM_ITERATIONS)),
            (lambda: get_sensor_data(cs, stop_event, sensor_data)),
        ]]
        print("Starting plot")
        do_plot(sensor_data)
        print("Shutting down...")
        stop_event.set()
    # print results and errors
    for f in futures:
        try:
            print(f"result: {f.result()}")
        except:
            print_exc()


if __name__ == '__main__':
    main()
