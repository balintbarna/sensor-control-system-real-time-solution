from concurrent.futures import ThreadPoolExecutor
from functools import cache
from queue import Empty, Queue
from threading import Event
from traceback import print_exc
#
from cffi import FFI
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation


class ControlSystem:
    def run(self):
        self.lib = self.load_native_module()
        self.pointer = self.lib.ControlSystem_new()
        self.lib.ControlSystem_run(self.pointer)
        # TODO release C object
        del self.pointer

    def stop(self):
        self.lib.ControlSystem_stop(self.pointer)

    def get_temperature(self) -> int:
        return self.lib.ControlSystem_get_temperature(self.pointer)

    def get_pressure(self) -> int:
        return self.lib.ControlSystem_get_pressure(self.pointer)

    @staticmethod
    @cache  # run only once
    def load_native_module():
        ffi = FFI()
        # Load the shared library
        lib = ffi.dlopen("./control_system.dll")
        # Define the C functions that we want to use
        ffi.cdef("""
                void* ControlSystem_new();
                void ControlSystem_run(void*);
                void ControlSystem_stop(void*);
                int ControlSystem_get_temperature(void*);
                int ControlSystem_get_pressure(void*);
                """)
        return lib


def get_sensor_data(cs: ControlSystem, stop_event: Event, queue: Queue):
    while not stop_event.is_set():
        try:
            temperature = cs.get_temperature()
            pressure = cs.get_pressure()
        except AttributeError:
            print("Control System not yet available")
        else:
            queue.put((temperature, pressure))
        stop_event.wait(0.1)


def do_plot(queue: Queue, temperature_setpoint = 20, pressure_setpoint = 1):
    fig, ax = plt.subplots()
    # Initialize the data and line objects
    x_data, temp_data, pressure_data = [], [], []
    temp_line = ax.plot(x_data, temp_data, label="Temperature")[0]
    pressure_line = ax.plot(x_data, pressure_data, label="Pressure")[0]
    # Add horizontal lines for the setpoints
    ax.axhline(y=temperature_setpoint, color="r",
               linestyle="--", label="Temperature Setpoint")
    ax.axhline(y=pressure_setpoint, color="g",
               linestyle="--", label="Pressure Setpoint")
    def update_plot(frame_index):
        try:
            temperature, pressure = queue.get_nowait()
            x_data.append(frame_index)
            temp_data.append(temperature)
            pressure_data.append(pressure)
            temp_line.set_data(x_data, temp_data)
            pressure_line.set_data(x_data, pressure_data)
            ax.relim()
            ax.autoscale_view()
            return temp_line, pressure_line
        except Empty:
            return temp_line, pressure_line

    def infinite_counter(start=0):
        yield start
        yield from infinite_counter(start + 1)

    _ = FuncAnimation(
        fig,
        update_plot,
        frames=infinite_counter(),
        cache_frame_data=False,
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
            (lambda: cs.run()),
            (lambda: get_sensor_data(cs, stop_event, sensor_data)),
        ]]
        print("Starting plot")
        do_plot(sensor_data)
        print("Shutting down...")
        cs.stop()
        stop_event.set()
    # print results and errors
    for f in futures:
        try:
            print(f"result: {f.result()}")
        except:
            print_exc()


if __name__ == '__main__':
    main()
