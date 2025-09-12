package org.simulation;

import java.util.Locale;

import org.models.Simulation;

public class Main {

    public static void main(String[] args) {
        Locale.setDefault(Locale.US);

        double heightSecondBox = 0.03;
        int particlesCount = 200;

        Simulation simulation = new Simulation(heightSecondBox, particlesCount);
        simulation.runSimulation(2000, String.format("initial_state_%.2f.csv", heightSecondBox));
    }

}
