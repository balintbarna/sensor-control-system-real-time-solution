#include <atomic>
#include <iostream>
#include <thread>
#include <chrono>
#include <random>
//
#include "threading.hpp"


// adjust delays here
#define SIM_DELAY std::chrono::milliseconds(100)
#define CONTROL_DELAY std::chrono::milliseconds(300)


#define BUFFER_T std::atomic<double>
#define STATE_T std::atomic<int>
#define STOPPED 0
#define RUNNING 1
#define PAUSED 2


template <typename F>
auto loopify(STATE_T &run_state, STATE_T &cycle_ms, F&& f)
{
    return [&] {
        while (run_state != STOPPED)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms));
            if (run_state == PAUSED) continue;
            f();
        }
    };
}


class Sensor
{
private:
    double data;

public:
    Sensor(double initial_value) : data(initial_value) {}

    void update(double value)
    {
        data = value;
    }

    void operator()(BUFFER_T &buffer)
    {
        buffer = data;
    }
};


class PIDController
{
private:
    double kp, ki, kd;
    double setpoint;
    double prev_error;
    double integral;

public:
    PIDController(double kp, double ki, double kd, double setpoint = 0.0) : kp(kp), ki(ki), kd(kd), setpoint(setpoint), prev_error(0), integral(0) {}

    void setSetpoint(double sp) {
        setpoint = sp;
    }

    double control(double measurement, double dt = 0.1)
    {
        double error = setpoint - measurement;
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }

    void operator()(BUFFER_T &control_buffer, BUFFER_T &sensor_buffer)
    {
        control_buffer = control(sensor_buffer);
    }
};


class ControlSystem
{
public:
    void operator()(const double target_temperature, const double target_pressure)
    {
        // System components
        Sensor temperature_sensor(20.0); // Celsius
        Sensor pressure_sensor(1.0); // atmospheres;
        PIDController temperatureController(1.0, 0.2, 0.0, target_temperature);
        PIDController pressureController(1.0, 0.2, 0.03, target_pressure);
        // Setup for random fluctuation
        std::default_random_engine generator;
        std::normal_distribution<double> temperature_distribution(2.0, 2.0);
        std::normal_distribution<double> pressure_distribution(0.5, 0.5);
        // Ready to start
        run_state = RUNNING;
        {
            ThreadPool execute(10);
            // start sensor reader threads
            execute(loopify(run_state, sensor_cycle_ms, [&]{
                temperature_sensor(measured_temperature);
            }));
            execute(loopify(run_state, sensor_cycle_ms, [&]{
                pressure_sensor(measured_pressure);
            }));
            std::this_thread::sleep_for(SIM_DELAY);
            // start sim
            execute(loopify(run_state, sim_cycle_ms, [&]{ 
                const auto temperature_fluctuation = temperature_distribution(generator);
                temperature_sensor.update(
                    measured_temperature
                    + -0.1 * (measured_temperature - 20)
                    + temperature_fluctuation + control_temperature);
            }));
            execute(loopify(run_state, sim_cycle_ms, [&]{
                const auto pressure_fluctuation = pressure_distribution(generator);
                pressure_sensor.update(
                    measured_pressure
                    + 0.05 * (measured_temperature - measured_pressure)
                    + pressure_fluctuation + control_pressure);
            }));
            std::this_thread::sleep_for(CONTROL_DELAY);
            // start controllers
            execute(loopify(run_state, controller_cycle_ms, [&]{
                temperatureController(control_temperature, measured_temperature);
            }));
            execute(loopify(run_state, controller_cycle_ms, [&]{
                pressureController(control_pressure, measured_pressure);
            }));
            // exits when all threads in threadpool have exited
        }
        // would be nice to print results from futures as they complete
    }

    void stop()
    {
        run_state = STOPPED;
    }

    void toggle_pause()
    {
        if (run_state == PAUSED) run_state = RUNNING;
        else run_state = PAUSED;
    }

    int get_state()
    {
        return run_state;
    }

    double get_temperature()
    {
        return measured_temperature;
    }

    double get_pressure()
    {
        return measured_pressure;
    }

private:
    STATE_T run_state = STOPPED;
    STATE_T sensor_cycle_ms = 10;
    STATE_T sim_cycle_ms = 100;
    STATE_T controller_cycle_ms = 10;
    BUFFER_T measured_temperature;
    BUFFER_T measured_pressure;
    BUFFER_T control_temperature;
    BUFFER_T control_pressure;
};

extern "C"
{
    ControlSystem *ControlSystem_new() { return new ControlSystem(); }
    void ControlSystem_delete(ControlSystem *cs) { delete cs; }
    void ControlSystem_run(ControlSystem *cs, double temperature, double pressure) { (*cs)(temperature, pressure); }
    void ControlSystem_stop(ControlSystem *cs) { cs->stop(); }
    void ControlSystem_toggle_pause(ControlSystem *cs) { cs->toggle_pause(); }
    int ControlSystem_get_state(ControlSystem *cs) { return cs->get_state(); }
    double ControlSystem_get_temperature(ControlSystem *cs) { return cs->get_temperature(); }
    double ControlSystem_get_pressure(ControlSystem *cs) { return cs->get_pressure(); }
}
