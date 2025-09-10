package org.simulation;

import org.models.Simulation;

public class Main {

    public static void main(String[] args) {
        double heightSecondBox = 0.06;
        int particlesCount = 250;

        Simulation simulation = new Simulation(heightSecondBox, particlesCount);
        simulation.runSimulation(100, "initial_state.csv");
    }

}
