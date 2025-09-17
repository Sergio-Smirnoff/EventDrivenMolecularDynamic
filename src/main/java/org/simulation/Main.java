package org.simulation;

import java.util.Locale;

import org.models.Simulation;

public class Main {

    public static void main(String[] args) {
        Locale.setDefault(Locale.US);

        double heightSecondBox = 0.04;
        int particlesCount = 250;

        Simulation simulation = new Simulation(heightSecondBox, particlesCount);
        simulation.runSimulation(100, String.format("initial_state_%.2f.csv", heightSecondBox));
    }

}
