# Formula SAE Electric Vehicle Simulation

This project is a physics-based simulation of a Formula SAE electric race car, written in Python using Pygame and Matplotlib. It models vehicle acceleration, power limiting, and driver controls, allowing you to tune parameters and visualize performance such as 0-60 mph times.

## Features
- Realistic vehicle physics (mass, torque, gear ratio, drag, friction)
- Power limiting logic (FSAE 80kW rule, with PID control)
- Driver input: throttle, brake, and steering (keyboard controlled)
- Real-time plots of speed, throttle, and brake
- 0-60 mph timer for benchmarking acceleration
- Data logging to CSV for further analysis

## Controls
- **W**: Throttle (hold for acceleration)
- **S**: Brake
- **A/D**: Steering left/right

## Getting Started
### Prerequisites
- Python 3.8+
- `pygame`, `matplotlib`

Install dependencies:
```bash
pip install pygame matplotlib
```

### Running the Simulation
```bash
python main.py
```
A window will open showing the car, real-time plots, and on-screen info. Use the controls above to drive the car.

## Configuration
Edit `car_config.py` to match your car's specs:
```python
MASS                 = 180        # kg (car + driver)
WHEEL_RADIUS         = 0.254      # m (10 in)
GEAR_RATIO           = 6.0        # final-drive * diff
MOTOR_MAX_TORQUE     = 220        # Nm (peak)
MOTOR_MAX_RPM        = 6500       # rpm (peak power)
PACK_NOMINAL_VOLTAGE = 830        # V
PACK_SAG_RESISTANCE  = 0.015      # Ω
```

Edit `PowerLimiter.py` for power rules:
```python
self.target_kw = 80  # FSAE rule (max 80kW at wheels)
```

## Tuning for Realism
- Use your motor's **peak torque** for sprints, **continuous torque** for endurance.
- Adjust **gear ratio** for your desired top speed and acceleration.
- Lower **rolling friction** and **air drag** for less resistance.
- Use the 0-60 mph timer to benchmark changes.

## Data Logging
Simulation results are logged to a CSV file (e.g., `vehicle_data_YYYYMMDD_HHMMSS.csv`) for post-run analysis.

## Troubleshooting
- If 0-60 mph is too slow, check:
  - Power limiter settings
  - Throttle input (hold W)
  - Motor torque and gear ratio
  - Vehicle mass and friction/drag
- Use debug prints in `vehicle.py` to see torque, power, and acceleration in real time.

## License
MIT License

## Credits
- Formula SAE community for inspiration
- Open source Python libraries: Pygame, Matplotlib 