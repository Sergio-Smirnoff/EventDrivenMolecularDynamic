package org.simulation;

import org.models.Simulation;

public class Main {

    public static void main(String[] args) {
        double heightSecondBox = 0.09;
        int particlesCount = 1;

        Simulation simulation = new Simulation(heightSecondBox, particlesCount);
        simulation.runSimulation(1000, "./initial_state.csv");
    }

}
