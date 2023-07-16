#include <iostream>
#include <thread>
#include <chrono>
#include <random>

class Sensor
{
private:
    double data;

public:
    Sensor() : data(0) {}

    void update(double value)
    {
        data = value;
    }

    double get_data() const
    {
        return data;
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
    PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), setpoint(0), prev_error(0), integral(0) {}

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
};

#define STOPPED 0
#define RUNNING 1
#define PAUSED 2

class ControlSystem
{
private:
    Sensor temperatureSensor;
    Sensor pressureSensor;
    PIDController temperatureController;
    PIDController pressureController;
    int run_state;

public:
    ControlSystem() :
        temperatureSensor(),
        pressureSensor(),
        temperatureController(1.0, 0.2, 0.0),
        pressureController(1.0, 0.2, 0.03),
        run_state(STOPPED) {}

    void run(const double target_temperature, const double target_pressure)
    {
        // setup for random fluctuation
        std::default_random_engine generator;
        std::normal_distribution<double> temperature_distribution(2.0, 2.0);
        std::normal_distribution<double> pressure_distribution(0.5, 0.5);
        // Set initial values in ...
        temperatureSensor.update(20.0); // Celsius
        pressureSensor.update(1.0); // atmospheres
        // Set target values
        temperatureController.setSetpoint(target_temperature);
        pressureController.setSetpoint(target_pressure);
        //
        run_state = RUNNING;
        //
        while (run_state != STOPPED)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (run_state == PAUSED) continue;

            // Update temperature based on previous temperature, random fluctuations, and control input
            const auto measured_temperature = temperatureSensor.get_data();
            const auto temperature_fluctuation = temperature_distribution(generator);
            const auto control_input_temperature = temperatureController.control(measured_temperature);
            temperatureSensor.update(measured_temperature + -0.1 * (measured_temperature - 20) + temperature_fluctuation + control_input_temperature);

            // Update pressure based on temperature (assuming ideal gas), random fluctuations, and control input
            const auto measured_pressure = pressureSensor.get_data();
            const auto pressure_fluctuation = pressure_distribution(generator);
            const auto control_input_pressure = pressureController.control(measured_pressure);
            pressureSensor.update(measured_pressure + 0.05 * (measured_temperature - measured_pressure) + pressure_fluctuation + control_input_pressure);
        }
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
        return temperatureSensor.get_data();
    }

    double get_pressure()
    {
        return pressureSensor.get_data();
    }
};

extern "C"
{
    ControlSystem *ControlSystem_new() { return new ControlSystem(); }
    void ControlSystem_delete(ControlSystem *cs) { delete cs; }
    void ControlSystem_run(ControlSystem *cs, double temperature, double pressure) { cs->run(temperature, pressure); }
    void ControlSystem_stop(ControlSystem *cs) { cs->stop(); }
    void ControlSystem_toggle_pause(ControlSystem *cs) { cs->toggle_pause(); }
    int ControlSystem_get_state(ControlSystem *cs) { return cs->get_state(); }
    double ControlSystem_get_temperature(ControlSystem *cs) { return cs->get_temperature(); }
    double ControlSystem_get_pressure(ControlSystem *cs) { return cs->get_pressure(); }
}
