# Greenhouse Temperature Control System Simulation

This project uses MATLAB to model the thermodynamics of a greenhouse and simulate different control strategies for maintaining its internal temperature. It compares a simple **Relay (ON/OFF) controller** against a **PID (Proportional-Integral-Derivative) controller** to determine which is more accurate and energy-efficient.

The simulation is based on the thermodynamic principles outlined in Equation (1) of the reference research paper, using realistic environmental data for Genoa, Italy, including fluctuating external temperature, solar radiation, and wind speed[cite: 1].

---

## 🎯 Core Questions Addressed

This simulation was designed to answer the following questions:

* How can the complex thermal behavior of a greenhouse be modeled and simplified for control system design?
* How well does a simple, low-cost Relay (ON/OFF) controller maintain a stable temperature inside the greenhouse?
* How does a more advanced PID controller compare to the Relay controller in terms of temperature stability and tracking a setpoint?
* Which control strategy consumes less energy to operate the greenhouse's heat pump?
* What are the trade-offs between implementation simplicity, control accuracy, and energy efficiency?

---

## ✨ Key Features

**Continuous and Discrete Modeling:** The script first implements a continuous-time model of the greenhouse using Euler integration and then creates a more robust discrete-time state-space model for the controllers.
**Relay Controller:** A simple ON/OFF controller with hysteresis (`±0.5°C`) is implemented to prevent rapid switching.
* **PID Controller:** A PID controller is automatically tuned using MATLAB's `pidtune` function for optimal performance. The controller's output is saturated to respect the heat pump's physical limits.
* **Two Control Scenarios:**
    1.  **Constant Setpoint:** Maintaining a steady `20°C.
    2.  **Variable Setpoint:** A more realistic scenario, aiming for `35°C` during the day (8 AM - 5 PM) and `15°C` at night.
* **Performance Analysis:** The script automatically calculates and compares the **Quadratic Deviation** (a measure of tracking accuracy) and total **Energy Consumption** (in kWh) for each controller.

---

## 📊 Results Summary

The simulation clearly shows that the **PID controller significantly outperforms the Relay controller** in every key metric.


* **Accuracy:** The PID controller maintains the greenhouse temperature much closer to the setpoint, with minimal overshoot and oscillation.The Relay controller constantly cycles above and below the setpoint.
* **Energy Efficiency:** By using proportional power adjustments instead of just ON/OFF, the PID controller uses the heat pump more efficiently, resulting in lower overall energy consumption.
* **Equipment Health:** The smooth control signal from the PID controller would cause less mechanical wear on the heat pump compared to the frequent, aggressive switching of the Relay controller.

For the variable temperature scenario, the analysis shows:
* **PID Control:** 98.6% better temperature tracking and 37.0% less energy usage compared to the Relay controller.

---

## 🚀 How to Run

1.  **Prerequisites:** You must have **MATLAB** installed.
2.  **Clone the Repository:** Download or clone this repository to your local machine.
3.  **Run the Script:** Open the `Part1.m` file in MATLAB and click the **Run** button.
4.  **View Results:** The script will automatically generate five plot windows showing the simulation results and print a detailed performance summary in the MATLAB Command Window.
   
---

## 📚 Reference

The model implemented in this simulation is based on the work presented in:

> Ben Ali, R., Aridhi, E., & Mami, A. (2015, December 21-23). *Dynamic model of an agricultural greenhouse using Matlab-Simulink environment* [Conference presentation]. 16th international conference on Sciences and Techniques of Automatic control & computer engineering - STA'2015, Monastir, Tunisia.
