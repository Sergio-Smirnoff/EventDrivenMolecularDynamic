package org.models;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.PriorityQueue;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;



public class Simulation {

    private final double heightFirstBox = 0.09;
    private final double width = 0.09;
    private double heightSecondBox = 0.09; // this is a variable // Considerar L = 0,03; 0,05; 0,07 y 0,09 m.
    private final double topWallB;
    private final double bottomWallB;

    private final Logger logger = LoggerFactory.getLogger(Simulation.class);

    private final double ballVelocity = 0.01; // in m/s
    private final double ballRadius = 0.0015; // meters

    private int particlesCount = 1000;


    private double totalTime = 0;
    private double currentTime = 0; //in seconds
    private final double timeStep = 0.1; // in seconds


    private final List<Particle> particles = new ArrayList<>(particlesCount);

    private final List<Double> collisionBoxA = new ArrayList<>();
    private final List<Double> collisionBoxB = new ArrayList<>();


    public Simulation(double heightSecondBox, int particlesCount) {
        this.heightSecondBox = heightSecondBox;
        this.particlesCount = particlesCount;
        this.topWallB = (heightFirstBox + heightSecondBox) / 2;
        this.bottomWallB = (heightFirstBox - heightSecondBox) / 2;
    }


    public double randomDistanceNumber(double min, double max) {
        double diff = max - min;
        return min + Math.random() * diff;
    }

    // Get a random angle for the particle's initial velocity
    private double getRandomAngle() {
        return Math.random() * 2.0 * Math.PI;
    }

    private double[] calculatePressures() {
        double[] totalPressure = {0,0};
        for (double pressure: collisionBoxA){
            totalPressure[0] += pressure;
        }
        collisionBoxA.clear();

        for (double pressure: collisionBoxB){
            totalPressure[1] += pressure;
        }
        collisionBoxB.clear();

        return totalPressure;
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

    private void saveSimulationState(String filePath) {
        // Logic to save the simulation state to a file
        Locale.setDefault(Locale.US);
        try (PrintWriter writer = new PrintWriter(new FileWriter(filePath))) {
            // Write headers
            writer.println("N: " + particlesCount);
            writer.println("L: " + heightSecondBox);

            double[] pressures = calculatePressures();
            // TODO: check if for moment 0 is necesary to add the pressures that will be 0
            writer.printf("%g;%g;%g\n", totalTime, pressures[0], pressures[1]); // fix with pressure

            // Write particle data
            writer.printf("positionX;positionY;velocityX;velocityY\n");
            for (Particle particle : particles) {
                writer.printf("%f;%f;%f;%f%n",
                        particle.getBallPositionX(),
                        particle.getBallPositionY(),
                        particle.getBallVelocityX(),
                        particle.getBallVelocityY());
            }

        } catch (IOException e) {
            logger.error(e.getMessage());
        }

    }

    /**
     * Generic function for calculating time taken to reach certain position
     *
     * @param initialPosition describes the starting point of the object.
     * @param finalPosition   describes the final destination of the object.
     * @param velocity        describes the velocity of the object.
     * @return time it takes to reach the destination
     */
    private double timeToPosition(double initialPosition, double finalPosition, double velocity) {
        return (finalPosition - initialPosition) / velocity;
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

        if (particle.getBallVelocityY() > 0) {
            double topWall = particle.getBallPositionX() < width ? heightFirstBox : topWallB;
            finalPosition = topWall - ballRadius;
        } else {
            double bottomWall = particle.getBallPositionX() < width ? 0 : bottomWallB;
            finalPosition = bottomWall + ballRadius;
        }
        double time = timeToPosition(y, finalPosition, particle.getBallVelocityY());
        return new Collision(time, particle, null, Wall.HORIZONTAL);
    }


    /**
     * Auxiliary function to calculate if particle is at height of opening of boxB.
     *
     * @param time     time
     * @param velocity particle velocity
     * @return if the particle, at that time, will reach opening height of boxB
     */
    private boolean atOpeningHeight(double y0, double time, double velocity) {
        double yf = y0 + velocity * time;
        return yf >= bottomWallB + ballRadius && yf <= topWallB - ballRadius;
    }


    /**
     * Function that calculates the time it takes for a particle to collide with vertical walls.
     * If the particle collides with the border wall at height of the opening of boxB then
     * the collision is discarded, since it's not a real collision.
     *
     *
     * if i discard collision then i should discard the other collision as well,
     * the horizontal corresponding
     *
     *
     * @param particle describes the particle to study
     * @return the new Collision item set to the corresponding wall
     */
    private Collision VerticalWallCollision(Particle particle) {
        double finalPosition;
        double x = particle.getBallPositionX();

        if (particle.getBallVelocityX() > 0) {
            double rightWall = x < width ? width : width * 2;
            finalPosition = rightWall - ballRadius;
        } else {
            double leftWall = x < width ? 0 : width;
            finalPosition = leftWall + ballRadius;
        }

        double time = timeToPosition(x, finalPosition, particle.getBallVelocityX());
        logger.debug("time {}",time);

        if (atOpeningHeight(particle.getBallPositionY(), time, particle.getBallVelocityY())) {
            return null;
        }

        return new Collision(time, particle, null, Wall.VERTICAL);
    }

    private Collision calculateParticlesCollision(Particle p1, Particle p2) {
        // Logic for mass collision detection and response goes here

        double time = 0;
        double dvx = ballVelocity * p2.getBallPositionX() - ballVelocity * p1.getBallPositionX();
        double dvy = ballVelocity * p2.getBallPositionY() - ballVelocity * p1.getBallPositionY();
        double drx = p2.getBallPositionX() - p1.getBallPositionX();
        double dry = p2.getBallPositionY() - p1.getBallPositionY();

        double calc = calculateDvDr(p1, p2);
        double dv = Math.pow(dvx, 2.0) + Math.pow(dvy, 2.0);
        double o = p1.getBallRadius() + p2.getBallRadius();
        double d = Math.pow(calc,2.0)-(dv +(Math.pow(drx, 2.0)+Math.pow(dry, 2.0))-Math.pow(o,2.0));

        // slide 14
        if ( calc >= 0 || d < 0){
            return null;
        }
        time = -(calc + Math.sqrt(d))/ dv;
        logger.debug("time: {}", time);

        return new Collision(time, p1, p2, null);
    }

    private void calculateInitialCollisions() {

        for (int i = 0; i < particles.size(); i++) {
            Particle p1 = particles.get(i);

            // Collisions with walls
            Collision collision = VerticalWallCollision(p1);
            if(collision != null) {
                p1.addCollision(collision);
            }
            p1.addCollision(HorizontalWallCollision(p1));


            // Collisions with other particles
            for (int j = i + 1; j < particles.size(); j++) {
                Particle p2 = particles.get(j);
                Collision particleCollision = calculateParticlesCollision(p1, p2);
                p1.addCollision(particleCollision);
                p2.addCollision(particleCollision);
            }
        }
    }


    /**
     * Finds the first collision that will occur
     *
     * @return collision that will first occur
     */
    private Collision nextCollision() {
        Collision collisionToOccur = particles.getFirst().getNextCollision();

        for (int i = 1; i < particles.size(); i++) {
            Collision collision = particles.get(i).getNextCollision();
            if ( collision != null && collisionToOccur != null && collision.getTime() < collisionToOccur.getTime()) {
                collisionToOccur = collision;
            }
        }
        return collisionToOccur;
    }


    // delta V = vxj - vxi , vyj -vyi
    // delta r = xj-xi , yj - yi
    // delta v * delta r = vxj - vxi *  xj-xi + vyj -vyi *  yj - yi
    // o = ri + rj
    private double calculateDvDr(Particle particleA, Particle particleB) {
        double dvx = ballVelocity * particleB.getBallPositionX() - ballVelocity * particleA.getBallPositionX();
        double dvy = ballVelocity * particleB.getBallPositionY() - ballVelocity * particleA.getBallPositionY();
        double drx = particleB.getBallPositionX() - particleA.getBallPositionX();
        double dry = particleB.getBallPositionY() - particleA.getBallPositionY();
        return (dvx * drx + dvy * dry);
    }

    private double calculateCollisionImpulse(Particle particleA, Particle particleB) {
        return  calculateDvDr(particleA,particleB)/ (particleA.getBallRadius() + particleB.getBallRadius());
    }


    private void updatePositions(double time) {
        for (Particle particle : particles) {
            double newX = particle.getBallPositionX() + particle.getBallVelocityX() * time;
            double newY = particle.getBallPositionY() + particle.getBallVelocityY() * time;
            particle.setBallPosition(newX, newY);

            // update collision times
            for (Collision collision : particle.getCollisions()) {
                collision.setTime(collision.getTime() - time);
            }

        }
    }

    private void removeOtherParticleCollisions(PriorityQueue<Collision> collisions, int id) {
        for (Collision collision : collisions) {
            collision.getParticleB().getCollisions().removeIf((col) -> col.getId() == id);
        }
    }


    /**
     * Makes the collision of a particle with the wall, updating the collisionBox list and the velocity of the particle
     *
     * @param particle
     * @param wall
     */
    // TODO: view how to calculate the impulse with the walls
    private void makeWallCollision(Particle particle, Wall wall, double time) {

        // Assuming that when a particle collides with the wall there are no particles to update
        particle.clearCollisions();

        updatePositions(time);
        // update velocities
        if (wall == Wall.HORIZONTAL) {
            particle.setBallVelocity(particle.getBallVelocityX(), -particle.getBallVelocityY());
            if (particle.isBoxA()) {
                collisionBoxA.add(calculateCollisionPressure(particle));
            } else {
                collisionBoxB.add(calculateCollisionPressure(particle));
            }

        } else if (wall == Wall.VERTICAL) {
            particle.setBallVelocity(-particle.getBallVelocityX(), particle.getBallVelocityY());
            if (particle.isBoxA()) {
                collisionBoxA.add(calculateCollisionPressure(particle));
            } else {
                collisionBoxB.add(calculateCollisionPressure(particle));
            }
        } else {
            logger.error("Invalid Wall Collision");
        }

    }

    private void makeParticleCollision(Particle particleA, Particle particleB, double time) {

        removeOtherParticleCollisions(particleA.getCollisions(), particleA.getId());
        removeOtherParticleCollisions(particleB.getCollisions(), particleB.getId());

        // Assuming that when a particle collides with the wall there are no particles to update
        particleA.clearCollisions();
        particleB.clearCollisions();

        updatePositions(time);

        // update velocities
        double impulse = calculateCollisionImpulse(particleA, particleB);
        // TODO: complete




    }

    /**
     * Function that will search for the closest collision and with change the collision list
     */
    private void makeCollision() {

        Collision nextCollision = nextCollision();

        // update simulation time
        if(nextCollision == null){
            return;
        }

        currentTime += nextCollision.getTime();
        logger.info("Current time: {}", currentTime);

        // need to check for wall
        Particle particleA = particles.get(nextCollision.getParticleA().getId());

        if (nextCollision.getWall() != null) {
            makeWallCollision(particleA, nextCollision.getWall(), nextCollision.getTime());

        } else if (nextCollision.getParticleB() != null) {
            // TODO: comment to view functionality of the wall collisions
            //makeParticleCollision(particleA, nextCollision.getParticleB(), nextCollision.getTime());
        } else {
            logger.error("next collision was null");
        }

    }

    private void initializeSystem() {
        for (int i = 0; i < particlesCount; i++) {
            // randomize positions and velocities
            double ballPositionX = randomDistanceNumber(ballRadius, width - ballRadius);
            double ballPositionY = randomDistanceNumber(ballRadius, heightFirstBox - ballRadius);
            double ballVelocityX = ballVelocity * Math.cos(getRandomAngle());
            double ballVelocityY = ballVelocity * Math.sin(getRandomAngle());

            Particle particle = new Particle(i, ballPositionX, ballPositionY, ballVelocityX, ballVelocityY);

            particles.add(particle);

        }
    }

    private double calculateCollisionPressure(Particle p){
        return 0;
    }

    public void runSimulation(int times, String filepath) {

        // initialize particles
        initializeSystem();
        saveSimulationState(filepath);

        calculateInitialCollisions();

        int current = 0;

        while (current < times) {
            if (currentTime/timeStep > 1) {
                current++;
                currentTime = 0;
                totalTime+=timeStep;
                saveSimulationState(filepath);
            }
            makeCollision();
        }
    }
}
