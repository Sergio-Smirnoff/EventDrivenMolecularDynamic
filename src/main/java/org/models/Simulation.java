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

    private int particlesCount = 250;


    private double totalTime = 0; // in seconds


    private final List<Particle> particles = new ArrayList<>(particlesCount);


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


    /*
     * Saves the current state of the simulation to a file.
     * Save in csv format
     * Headers --> N: particlesCount
     *         --> L: heightSecondBox
     * time;
     * Header for particles
     * positionX;positionY;velocityX;velocityY
     * Locale.setDefault(Locale.US); for writing the decimals with comma
     */

    private void saveSimulationState(String filePath, boolean printHeaders) {
        // Logic to save the simulation state to a file
        Locale.setDefault(Locale.US);
        try (PrintWriter writer = new PrintWriter(new FileWriter(filePath, !printHeaders))) { // append mode if not printing headers
            // Write headers
            if (printHeaders) {
                writer.println("N: " + particlesCount);
                writer.println("L: " + heightSecondBox);
            }
            writer.printf("%g;\n", totalTime); // fix with pressure

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


    private void calculateParticleCollisions(Particle particle) {

        // Collisions with walls
        Collision collisionV = VerticalWallCollision(particle);
        Collision collisionH = HorizontalWallCollision(particle);

        if (collisionV.getTime() >= collisionH.getTime()) {
            particle.addCollision(collisionH);
        } else {
            particle.addCollision(collisionV);
        }
        /* TODO: uncomment to enable particle collisions
        for (Particle other : particles) {
            if (other != particle) {
                collision = calculateParticlesCollision(particle, other);
                if (collision != null) {
                    particle.addCollision(collision);
                    other.addCollision(collision);
                }
            }
        }*/
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
        logger.debug("----- Horizontal Wall Collision ------");
        logger.debug("particle {}", particle.getId());
        logger.debug("velocity {}", particle.getBallVelocityY());
        logger.debug("initial position {}", particle.getBallPositionY());
        logger.debug("final position {}", finalPosition);
        logger.debug("time {}",time);
        logger.debug("-----------");
        return new Collision(time, particle.getId(), Wall.HORIZONTAL);
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
     * todo:if i have x < width but the particle has a little bit of itself in box B, is
     *      it in box A or in box B?
     *
     *
     *
     * @param particle describes the particle to study
     * @return the new Collision item set to the corresponding wall
     */
    private Collision VerticalWallCollision(Particle particle) {
        double finalPosition;
        double x = particle.getBallPositionX();
        boolean inBoxA = x < width;

        if (particle.getBallVelocityX() > 0) {
            double rightWall = inBoxA ? width : width * 2;
            finalPosition = rightWall - ballRadius;
        } else {
            double leftWall = inBoxA ? 0 : width;
            finalPosition = leftWall + ballRadius;
        }

        double time = timeToPosition(x, finalPosition, particle.getBallVelocityX());
        logger.debug("----- Vertical Wall Collision ------");
        logger.debug("particle {}", particle.getId());
        logger.debug("velocity {}", particle.getBallVelocityX());
        logger.debug("initial position {}", particle.getBallPositionX());
        logger.debug("final position {}", finalPosition);
        logger.debug("time {}",time);
        logger.debug("-----------");
        boolean atOpeningHeight = atOpeningHeight(particle.getBallPositionY(), time, particle.getBallVelocityY());

        if (atOpeningHeight && ((particle.getBallVelocityX() > 0 && inBoxA) || (particle.getBallVelocityX() < 0 && !inBoxA))) {
            return null;
        }

        return new Collision(time, particle.getId(), Wall.VERTICAL);
    }

    /**
     * Calculates ∆v∆r
     * @param dv is ∆v = (∆vx, ∆vy)
     * @param dr is ∆r = (∆rx, ∆ry)
     * @return product
     */
    private double calculateDvDr(double[] dv, double[] dr) {
        return (dv[0]* dr[0] + dv[1] * dr[1]);
    }

    /**
     * Calculates particles collision
     * @param p1 first particle
     * @param p2 second particle
     * @return Collision between said particles
     */

    //TODO: check calculation of time, giving negative values
    private Collision calculateCollisionBetweenParticles(Particle p1, Particle p2) {
        // Logic for mass collision detection and response goes here

        double time = 0;
        // ∆v = (∆vx, ∆vy)
        double[] dv = {0,0};
        dv[0] = p2.getBallVelocityX() - p1.getBallPositionX();
        dv[1] = p2.getBallPositionY() - p1.getBallPositionY();
        // ∆r = (∆rx, ∆ry)
        double[] dr = {0,0};
        dr[0] = p2.getBallPositionX() - p1.getBallPositionX();
        dr[1] = p2.getBallPositionY() - p1.getBallPositionY();

        double calc = (dv[0]* dr[0] + dv[1] * dr[1]);
        double dvdv = Math.pow(dv[0], 2.0) + Math.pow(dv[1], 2.0);
        double drdr = Math.pow(dr[0], 2.0) + Math.pow(dr[1], 2.0);
        double o = p1.getBallRadius() + p2.getBallRadius();
        double d = Math.pow(calc,2.0) - ( dvdv * (drdr - o) );

        // slide 14
        if ( calc >= 0 || d < 0){
            return null;
        }
        time = -(calc + Math.sqrt(d))/ dvdv;
        logger.debug("time: {}", time);

        return new Collision(time, p1.getId(), p2.getId(), null);
    }

    /**
     * Calculates all collisions for all particles
     */
    private void calculateInitialCollisions() {
        for (Particle p1 : particles) {
            calculateParticleCollisions(p1);
        }
    }


    /**
     * Finds the first collision that will occur
     *
     * @return collision that will first occur
     */
    private Collision firstCollision() {
        Collision collisionToOccur = particles.getFirst().getNextCollision();

        for (int i = 1; i < particles.size(); i++) {
            Collision collision = particles.get(i).getNextCollision();
            if ( collision != null && collisionToOccur != null && collision.getTime() < collisionToOccur.getTime()) {
                collisionToOccur = collision;
            }
        }
        return collisionToOccur;
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

    /**
     * Given a particle id and its collisions, removes mirror collision from the collision list of the other particle
     * Example:
     *          If A's collision List looks like (B, C, Wall), then for B and C remove collision with A
     *
     * @param collisions particle id collision list
     * @param id id of particle
     */
    private void removeOtherParticleCollisions(PriorityQueue<Collision> collisions, int id) {
        for (Collision collision : collisions) {
            if (collision.getWall() == null) {
                Particle particleB = particles.get(collision.getParticleB());
                particleB.getCollisions().removeIf((col) -> col.getParticleB() == id);
            }
        }
    }


    /**
     * Function that makes collision with a wall happen:
     * - updates particle velocity vector
     * - recalculates particle collision list
     *
     * @param particle given particle
     * @param wall collision wall
     */
    private void makeWallCollision(Particle particle, Wall wall, double time) {

        removeOtherParticleCollisions(particle.getCollisions(), particle.getId());
        particle.clearCollisions();
        updatePositions(time);
        // update velocities
        if (wall == Wall.HORIZONTAL) {
            particle.setBallVelocity(particle.getBallVelocityX(), -particle.getBallVelocityY());

        } else if (wall == Wall.VERTICAL) {
            particle.setBallVelocity(-particle.getBallVelocityX(), particle.getBallVelocityY());
        } else {
            logger.error("Invalid Wall Collision");
        }
        calculateParticleCollisions(particle);
    }

    // must also recalculate particles collisions
    private void makeParticleCollision(Particle particleA, Particle particleB, double time) {

        removeOtherParticleCollisions(particleA.getCollisions(), particleA.getId());
        removeOtherParticleCollisions(particleB.getCollisions(), particleB.getId());

        // Assuming that when a particle collides with the wall there are no particles to update
        particleA.clearCollisions();
        particleB.clearCollisions();
    }

    /**
     *  Function that makes general collision happen:
     *  - searches for the first collision that will occur among all collisions
     *  - updates all particle positions -> new system state given new system time
     *  - updates particle (or particles) velocity and collision list given such collision
     */
    private void makeCollision() {

        Collision nextCollision = firstCollision();

        if(nextCollision == null){
            return;
        }

        // updates system total time and particle state
        totalTime += nextCollision.getTime();
        logger.debug("Current time: {}", totalTime);

        // Updates particle velocity and collision list
        Particle particleA = particles.get(nextCollision.getParticleA());

        if (nextCollision.getWall() != null) {
            makeWallCollision(particleA, nextCollision.getWall(), nextCollision.getTime());

        } else {
            // TODO: comment to view functionality of the wall collisions
            //makeParticleCollision(particleA, nextCollision.getParticleB(), nextCollision.getTime());
        }
    }

    private void initializeSystem() {
        for (int i = 0; i < particlesCount; i++) {
            double ballPositionX;
            double ballPositionY;
            boolean overlaps;

            do {
                overlaps = false;

                ballPositionX = randomDistanceNumber(ballRadius, width - ballRadius);
                ballPositionY = randomDistanceNumber(ballRadius, heightFirstBox - ballRadius);

                for (Particle other : particles) {
                    double dx = ballPositionX - other.getBallPositionX();
                    double dy = ballPositionY - other.getBallPositionY();
                    double distance = Math.sqrt(dx * dx + dy * dy);

                    if (distance < 2 * ballRadius) {
                        overlaps = true;
                        break; 
                    }
                }
            } while (overlaps);
            double angle = getRandomAngle();
            double ballVelocityX = ballVelocity * Math.cos(angle);
            double ballVelocityY = ballVelocity * Math.sin(angle);
            Particle particle = new Particle(i, ballPositionX, ballPositionY, ballVelocityX, ballVelocityY);
            particles.add(particle);
        }
    }


    private double calculateCollisionPressure(Particle p){
        return 0;
    }

    public void runSimulation(int times, String filepath) {

        logger.info("Starting simulation with {} particles and box height {}", particlesCount, heightSecondBox);
        // initialize particles
        initializeSystem();
        logger.info("Initialized system with {} particles", particlesCount);
        saveSimulationState(filepath, true);

        logger.info("Collision number: 0");
        calculateInitialCollisions();
        logger.info("Calculated initial collisions");

        // iteration of collisions
        int current = 1;
        while (current++ < times) {
            logger.info("***********************");
            logger.info("Collision number: {}", current);
            makeCollision();
            logger.info("***********************");
            saveSimulationState(filepath, false);
        }
    }
}
