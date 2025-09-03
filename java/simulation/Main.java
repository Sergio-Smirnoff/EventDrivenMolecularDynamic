import models.*;

public class Main {

    public static void main(String[] args) {
        double heightSecondBox = 0.09;
        int particlesCount = 1000;

        Simulation simulation = new Simulation(heightSecondBox, particlesCount);
        simulation.runSimulation();
    }

}
