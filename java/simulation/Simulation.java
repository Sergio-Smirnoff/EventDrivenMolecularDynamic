package EventDrivenMolecularDynamic.java.simulation;

public class Simulation {

    private final double heightFirstBox = 0.09;
    private final double width = 0.09;
    private double heightSecondBox = 0.09; // this is a variable
    private final double ballMass = 1; // in kg
    private final double ballRadius = 0.0015; // meters
    private final double ballVelocity = 0.01; // in m/s
    private final int particlesCount = 1000;


    public Simulation(double heightSecondBox) {
        this.heightSecondBox = heightSecondBox;
    }

    private int calculateNextCollision() {
        // Collision detection and response logic for the second box goes here
        return 0;
    }


    /*
     * Saves the current state of the simulation to a file.
     * Save in csv format
     * time;positionX;positionY;velocityX;velocityY
     */

    private boolean saveSimulationState(String filePath) {
        // Logic to save the simulation state to a file
        return false;
    }

    // change the sign in X
    private int timeToVerticalWallCollision() {
        // Logic for vertical wall collision detection and response goes here
        return 0;
    }

    // change the sign in Y
    private int timeToHorizontalWallCollision() {
        // Logic for horizontal wall collision detection and response goes here
        return 0;
    }

    private int timeToMassCollision() {
        // Logic for mass collision detection and response goes here
        return 0;
    }

    private double calculateCollisionImpulse(){
        return 0;
    }

    public void runSimulation() {
        // Simulation logic goes here




    }

}
