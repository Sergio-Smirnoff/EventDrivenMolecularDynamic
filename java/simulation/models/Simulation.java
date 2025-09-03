package models;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.io.PrintWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

public class Simulation {

    private final double heightFirstBox = 0.09;
    private final double width = 0.09;
    private double heightSecondBox = 0.09; // this is a variable // Considerar L = 0,03; 0,05; 0,07 y 0,09 m.

    private final double ballVelocity = 0.01; // in m/s
    private final double ballRadius = 0.0015; // meters

    private int particlesCount = 1000;
    
    // implement logger with the good class

    private int currentTime = 0; //in seconds
    private final double timeStep = 0.01; // in seconds


    private final List<Particle> particles = new ArrayList<>(particlesCount);

    private final List<Double> collisionBoxA = new ArrayList<>();
    private final List<Double> collisionBoxB = new ArrayList<>();


    public Simulation(double heightSecondBox, int particlesCount) {
        this.heightSecondBox = heightSecondBox;
        this.particlesCount = particlesCount;
    }

    private int calculateNextCollision() {
        // Collision detection and response logic for the second box goes here
        return 0;
    }


    /*
     * Saves the current state of the simulation to a file.
     * Save in csv format
     * Headers --> N: particlesCount
     *         --> L: heightSecondBox
     * time;pressureA;pressureB
     * Header for particles
     * positionX;positionY;velocityX;velocityY
     * Locale.setDefault(Locale.US); for writing the decimals with comma
     */

    private boolean saveSimulationState(String filePath) {
        // Logic to save the simulation state to a file
        Locale.setDefault(Locale.US);
        try (PrintWriter writer = new PrintWriter(new FileWriter(filePath))) {
            // Write headers
            writer.println("N: " + particlesCount);
            writer.println("L: " + heightSecondBox);
            writer.printf("%d;pressureA;pressureB\n", currentTime); // fix with pressure

            // Write particle data
            writer.printf("positionX;positionY;velocityX;velocityY\n");
            for (Particle particle : particles) {
                writer.printf("%f;%f;%f;%f%n",
                        particle.getBallPositionX(),
                        particle.getBallPositionY(),
                        particle.getBallVelocityX(),
                        particle.getBallVelocityY());
            }

            return true;
        } catch (IOException e) {
            e.printStackTrace();
        }

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
    
    private double calculatePressure(String filepath){
        return 0;
    }


    public double randomDistanceNumber( double min, double max ){
        double diff = max - min;
        return min + Math.random() * diff;
    }

    // Get a random angle for the particle's initial velocity
    private double getRandomAngle(){
        return Math.random() * 2.0 * Math.PI;
    }

    private void initializeSystem() {
        for (int i = 0; i < particlesCount; i++) {
            // randomize positions and velocities
            double ballPositionX = randomDistanceNumber(ballRadius, width-ballRadius) * width;
            double ballPositionY = randomDistanceNumber(ballRadius, heightFirstBox-ballRadius) * heightFirstBox;
            double ballVelocityX = ballVelocity * Math.cos(getRandomAngle());
            double ballVelocityY = ballVelocity * Math.sin(getRandomAngle());

            Particle particle = new Particle( i, ballPositionX, ballPositionY, ballVelocityX, ballVelocityY );

            particles.add(particle);

        }

        // just for testing 
        // printing the initial state

        saveSimulationState("initial_state.txt");
    }

    public void runSimulation() {

        // initialize particles
        initializeSystem();

    }

}
