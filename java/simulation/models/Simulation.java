package models;

import java.util.ArrayList;
import java.util.List;
import java.io.PrintWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

public class Simulation {

    private final double heightFirstBox = 0.09;
    private final double width = 0.09;
    private double heightSecondBox = 0.09; // this is a variable // Considerar L = 0,03; 0,05; 0,07 y 0,09 m.
    private final double topWallB;
    private final double bottomWallB;


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
        this.topWallB = (heightFirstBox + heightSecondBox) / 2;
        this.bottomWallB = (heightFirstBox - heightSecondBox) / 2;
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

    /**
     * Generic function for calculating time taken to reach certain position
     * @param initialPosition describes the starting point of the object.
     * @param finalPosition describes the final destination of the object.
     * @param velocity describes the velocity of the object.
     * @return time it takes to reach the destination
     */
    private double timeToPosition(double initialPosition, double finalPosition, double velocity){
        return (finalPosition-initialPosition)/velocity;
    }


    /**
     * Function that calculates the time it takes for a particle to collide with Horizontal walls.
     *
     * @param particle describes the particle to study
     * @return the new Collision item set to the corresponding wall
     */
    private Collision HorizontalWallCollision(Particle particle) {
        double y = particle.getBallPositionY();
        double finalPosition;

        if(particle.getBallVelocityY() > 0){
            double topWall = particle.getBallPositionX() < width ? heightFirstBox : topWallB;
            finalPosition = topWall - ballRadius;
        }else{
            double bottomWall = particle.getBallPositionX() < width ? 0 : bottomWallB;
            finalPosition = bottomWall + ballRadius;
        }
        double time = timeToPosition(y, finalPosition, particle.getBallVelocityY());
        return new Collision(time, particle, null, Wall.HORIZONTAL);
    }


    /**
     * Auxiliary function to calculate if particle is at height of opening of boxB.
     * @param time time
     * @param velocity particle velocity
     * @return if the particle, at that time, will reach opening height of boxB
     */
    private boolean atOpeningHeight(double y0, double time, double velocity){
        double yf = y0 + velocity * time;
        return yf >= bottomWallB + ballRadius && yf <= topWallB - ballRadius;
    }


    /**
     * Function that calculates the time it takes for a particle to collide with vertical walls.
     * If the particle collides with the border wall at height of the opening of boxB then
     * the collision is discarded, since it's not a real collision.
     *
     * @param particle describes the particle to study
     * @return the new Collision item set to the corresponding wall
     */
    private Collision VerticalWallCollision(Particle particle) {
        double finalPosition;
        double x = particle.getBallPositionX();

        if(particle.getBallVelocityX() > 0){
            double rightWall = x < width ? width : width * 2;
            finalPosition = rightWall - ballRadius;
        }else{
            double leftWall = x < width ? 0 : width;
            finalPosition = leftWall + ballRadius;
        }

        double time = timeToPosition(x, finalPosition, particle.getBallVelocityX());

        if(atOpeningHeight(particle.getBallPositionY(), time, particle.getBallVelocityY())){
            return null;
        }

        return new Collision(time, particle, null, Wall.VERTICAL);
    }


    private void calculateCollisions(){
        // Clear previous collisions?
        for(int i = 0; i< particles.size(); i++){
            Particle p1 = particles.get(i);

            // Collisions with walls
            p1.addCollision(VerticalWallCollision(p1));
            p1.addCollision(HorizontalWallCollision(p1));

            // Collisions with other particles
            for(int j = i+1; j< particles.size(); j++){
                Particle p2 = particles.get(j);
                Collision particleCollision = ParticlesCollision(p1, p2);
                p1.addCollision(particleCollision);
                p2.addCollision(particleCollision);
            }
        }
    }

    private double minCollisionTime(){
        double tc = particles.getFirst().getNextCollision().getTime();

        for(int i = 1; i < particles.size(); i++){
            double time = particles.get(i).getNextCollision().getTime();
            if(time < tc) {
                tc = time;
            }
        }
        return tc;
    }

    private Collision ParticlesCollision(Particle p1, Particle p2) {
        // Logic for mass collision detection and response goes here
        return null;
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
