from concurrent.futures import ThreadPoolExecutor
from functools import cache
from queue import Empty, Queue
from threading import Event
from traceback import print_exc
from typing import Callable
#
from cffi import FFI
from funcy import ignore
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button


class ControlSystem:
    def __call__(self, target_temperature: float, target_pressure: float):
        self.lib = self.load_native_module()
        self.pointer = self.lib.ControlSystem_new()
        self.lib.ControlSystem_run(self.pointer,
                                   target_temperature,
                                   target_pressure)
        self.lib.ControlSystem_delete(self.pointer)
        del self.pointer

    def stop(self):
        self.lib.ControlSystem_stop(self.pointer)

    def toggle_pause(self):
        self.lib.ControlSystem_toggle_pause(self.pointer)

    def is_running(self) -> bool:
        return self.lib.ControlSystem_get_state(self.pointer) == 1

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
                void ControlSystem_delete(void*);
                void ControlSystem_run(void*, double, double);
                void ControlSystem_stop(void*);
                void ControlSystem_toggle_pause(void*);
                int ControlSystem_get_state(void*);
                double ControlSystem_get_temperature(void*);
                double ControlSystem_get_pressure(void*);
                """)
        return lib


def get_sensor_data(cs: ControlSystem, stop_event: Event, queue: Queue):
    while not stop_event.is_set():
        stop_event.wait(0.1)
        try:
            if not cs.is_running(): continue
            temperature = cs.get_temperature()
            pressure = cs.get_pressure()
        except AttributeError:
            print("Control System not available")
        else:
            queue.put((temperature, pressure))


def do_plot(*, queue: Queue, toggle_pause: Callable[[], None],
            temperature_setpoint: float, pressure_setpoint: float):
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
    pause_button = Button(fig.add_axes([0.7, 0.9, 0.2, 0.075]),
                          "Pause")
    def adv_toggle_pause(*a, **kw):
        toggle_pause()
        pause_button.label.set_text(
            "Resume" if pause_button.label.get_text() == "Pause" else "Pause")
    pause_button.on_clicked(adv_toggle_pause)
    def update_plot(frame_index):
        @ignore(IndexError, default=frame_index)
        def get_frame():
            return x_data[-1] + 1
        try:
            temperature, pressure = queue.get_nowait()
        except Empty:
            pass
        else:
            x_data.append(get_frame())
            temp_data.append(temperature)
            pressure_data.append(pressure)
            temp_line.set_data(x_data, temp_data)
            pressure_line.set_data(x_data, pressure_data)
            ax.relim()
            ax.autoscale_view()
        return temp_line, pressure_line

    def infinite_counter(start=0):
        while True:
            yield start
            start += 1

    _ = FuncAnimation(
        fig,
        update_plot,
        frames=infinite_counter(),
        cache_frame_data=False,
    )
    ax.legend()
    plt.show()


def main():
    TARGET_TEMPERATURE = 50.
    TARGET_PRESSURE = 2.5
    print("Starting...")
    control = ControlSystem()
    stop_event = Event()
    sensor_data = Queue()
    with ThreadPoolExecutor(max_workers=10) as exec:
        print("Starting backend")
        futures = [exec.submit(f) for f in [
            (lambda: control(TARGET_TEMPERATURE, TARGET_PRESSURE)),
            (lambda: get_sensor_data(control, stop_event, sensor_data)),
        ]]
        print("Starting plot")
        do_plot(queue=sensor_data,
                toggle_pause=control.toggle_pause,
                temperature_setpoint=TARGET_TEMPERATURE,
                pressure_setpoint=TARGET_PRESSURE)
        print("Shutting down...")
        control.stop()
        stop_event.set()
    # print results and errors
    for f in futures:
        try:
            print(f"result: {f.result()}")
        except:
            print_exc()


if __name__ == '__main__':
    main()
