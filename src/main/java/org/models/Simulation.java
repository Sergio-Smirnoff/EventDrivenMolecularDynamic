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
    private final double scaling = 100000.0;

    private final double heightFirstBox = 0.09;
    private final double width = 0.09;
    private final double heightSecondBox;
    private final double topWallB;
    private final double bottomWallB;

    private final Logger logger = LoggerFactory.getLogger(Simulation.class);

    private final double ballVelocity = 0.01; // in m/s
    private final double ballRadius = 0.0015; // meters
    private final static double EPS = 1e-12;

    private final int particlesCount;
    private double totalTime = 0; // in seconds
    private final List<Particle> particles;

    public Simulation(double heightSecondBox, int particlesCount) {
        this.heightSecondBox = heightSecondBox;
        this.particlesCount = particlesCount;
        this.topWallB = (heightFirstBox + heightSecondBox) / 2;
        this.bottomWallB = (heightFirstBox - heightSecondBox) / 2;
        this.particles = new ArrayList<>(particlesCount);
    }

    /* ----------------------- Main Simulation Loop Functions ----------------------- */

    public void runSimulation(int maxIterations, String filepath){
        logger.info("Starting Simulation with {}", particlesCount);
        initializeSystem();
        saveSimulationState(filepath, true, new ArrayList<>()); // save initial state
        calculateInitialCollisions();

        List<Particle> collisionedParticles = new ArrayList<>();

        for (int collisionCount = 1; collisionCount <= maxIterations; collisionCount++) {
            logger.info("***********************");
            logger.info("Collision number: {}", collisionCount);

            // 1. Find the very next event in the entire system.
            Collision nextEvent = findNextEvent();

            if (nextEvent == null || nextEvent.getTime() == Double.POSITIVE_INFINITY) {
                logger.warn("No more collisions can be predicted. Ending simulation.");
                break;
            }

            logger.info(" ");
            logger.info("colliding time {}", nextEvent.getTime());
            logger.info("colliding particle {}", nextEvent.getParticleA());
            if (nextEvent.getParticleB() != -1)
                logger.info("colliding with particle {}", nextEvent.getParticleB());
            else if (nextEvent.getWall() != null)
                logger.info("colliding with wall {}", nextEvent.getWall());
            logger.info("collision {}", nextEvent.isTrueCollision() ? "true":"false");
            logger.info(" ");

            // 3. Advance the system to the time of that event.
            if(!advanceSystem(nextEvent.getTime()))
                return;

            // 4. Resolve the collision and re-predict futures for involved particles.
            resolveCollision(nextEvent);

            // 5. Save the state of the system.
            if ( nextEvent.getParticleA() >= 0 && nextEvent.getParticleA() < particlesCount ) {
                collisionedParticles.add(particles.get(nextEvent.getParticleA()));
            }
            if ( nextEvent.getParticleB() >= 0 && nextEvent.getParticleB() < particlesCount ) {
                collisionedParticles.add(particles.get(nextEvent.getParticleB()));
            }

            saveSimulationState(filepath, false, collisionedParticles);
            collisionedParticles.clear();
        }
    }


    /**
     * Finds the next event that will occur
     *
     * @return collision that will occur next
     */
    private Collision findNextEvent() {
        Collision nextEvent = null;
        for (Particle p : particles) {
            if (!p.hasCollisions()) continue;
            Collision earliestForP = p.getNextCollision();
            if (nextEvent == null || earliestForP.getTime() < nextEvent.getTime()) {
                nextEvent = earliestForP;
            }
        }
        return nextEvent;
    }


    /**
     * Advances the system to the next event
     *
     * @param timeSkip Time to be fast forwarded
     */
    private boolean advanceSystem(double timeSkip){
        if(timeSkip < 0){
            logger.warn("Attempted to advance time by a negative value: {}. OJO.", timeSkip);
            return false;
        }

        for (Particle p : particles) {
            double newX = p.getBallPositionX() + p.getBallVelocityX() * timeSkip;
            double newY = p.getBallPositionY() + p.getBallVelocityY() * timeSkip;
            p.setBallPosition(newX, newY);
            for (Collision c : p.getCollisions()) {
                c.setTime(c.getTime() - timeSkip);
                if(c.collisionWithWall()){
                    logger.info("new collision with wall time {} for particle {}", c.getTime(), p.getId());
                }else{
                    logger.info("new collision time {} for particle {} against {}", c.getTime(), c.getParticleA(), c.getParticleB());
                }
            }
        }

        totalTime += timeSkip;
        logger.debug("Advanced time by {}. Total time: {}", timeSkip, totalTime);
        return true;
    }


    /* ----------------------- Collision Functions ----------------------- */

    // ojo con getParticleB porque no se si sería esa la partícula a chequear, depende de como se guarde la colisión espejada

    /**
     * Cleans given particle collisions.
     * If Particle A collides with B, C and a wall, it removes the A-B and A-C collision from B´s and C's list respectively
     * @param collidingParticle given particle to remove collisions
     */
    private void cleanCollisions(Particle collidingParticle){
        PriorityQueue<Collision> collisions = collidingParticle.getCollisions();
        for (Collision collision : collisions) {
            if (collision.getWall() == null) {
                if (collision.getParticleA() == collidingParticle.getId()) {
                    Particle particleB = particles.get(collision.getParticleB());
                    particleB.getCollisions().removeIf((col) -> col.getParticleA() == collidingParticle.getId());

                } else if (collision.getParticleB() == collidingParticle.getId()) {
                    Particle particleA = particles.get(collision.getParticleA());
                    particleA.getCollisions().removeIf((col) -> col.getParticleB() == collidingParticle.getId());
                }
            }
        }
        collidingParticle.clearCollisions();
    }



    /**
     * Given a particle that collides with a wall, or to the invisible wall, corrects
     * the value to avoid things like y = 0.00150000000000002 or y = 0.001499999999999999999996
     * If the particle happens to be in the edges of the opening (0.0885 or 0.0915) then
     * it leaves it as it is
     *
     * @param p particle which collides
     * @param wall wall to which the particle collides
     */
    private void snapToWall(Particle p, Wall wall){
        boolean inBoxA = p.getBallPositionX() < width - ballRadius - EPS;
        boolean inBoxB = p.getBallPositionX() > width + ballRadius + EPS;
        if(wall == Wall.VERTICAL){
            double newX = p.getBallPositionX();
            if(p.getBallVelocityX()>0){
                if(inBoxA){
                    newX = width - ballRadius;
                } else if (inBoxB) {
                    newX = 2 * width - ballRadius;
                }
                // x == 0.0885 or 0.0915
            }else{
                if(inBoxA){
                    newX = ballRadius;
                } else if (inBoxB) {
                    newX = width + ballRadius;
                }
            }
            p.setBallPosition(newX, p.getBallPositionY());
        }else{
            double newY = p.getBallPositionY();
            if(p.getBallVelocityY() > 0){
                if(inBoxA){
                    newY = heightFirstBox - ballRadius;
                } else if (inBoxB) {
                    newY = topWallB - ballRadius;
                }
            }else{
                if(inBoxA){
                    newY = ballRadius;
                } else if (inBoxB) {
                    newY = bottomWallB + ballRadius;
                }
            }
            p.setBallPosition(p.getBallPositionX(), newY);
        }
    }

    /**
     * Updates velocities of colliding particle depending on wall collision
     * @param p colliding particle
     * @param w colliding wall
     */
    private void handleWallBounce(Particle p, Wall w){
        if (w.equals(Wall.VERTICAL)) {
            p.setBallVelocity(-p.getBallVelocityX(), p.getBallVelocityY());
        } else {
            p.setBallVelocity(p.getBallVelocityX(), -p.getBallVelocityY());
        }
    }

        // check
    private void handleParticleCollision(Particle pA, Particle pB){
        double dx = pB.getBallPositionX() - pA.getBallPositionX();
        double dy = pB.getBallPositionY() - pA.getBallPositionY();
        double distance = Math.sqrt(dx * dx + dy * dy);

        // Normal vector
        double nx = dx / distance;
        double ny = dy / distance;

        // Relative velocity
        double dvx = pB.getBallVelocityX() - pA.getBallVelocityX();
        double dvy = pB.getBallVelocityY() - pA.getBallVelocityY();

        // Velocity along the normal
        double vn = dvx * nx + dvy * ny;

        // If particles are moving apart, do nothing
        if (vn > 0) return;

        // Impulse scalar
        double impulse = -2 * vn / 2; // assuming equal mass

        // Update velocities
        pA.setBallVelocity(pA.getBallVelocityX() - impulse * nx, pA.getBallVelocityY() - impulse * ny);
        pB.setBallVelocity(pB.getBallVelocityX() + impulse * nx, pB.getBallVelocityY() + impulse * ny);

        logger.info("Colliding particle {} with particle {}", pA.getId(), pB.getId());
    }

    /**
     * Resolves occurring collision. Analyses if wall collision or particle collision occur.
     * If particle collides with invisible wall, it only recalculates collision since particle
     * changes environment.
     * @param collision occurring collision
     */
    private void resolveCollision(Collision collision) {
        Particle particleA = particles.get(collision.getParticleA());
        if(collision.getWall() != null){
            snapToWall(particleA, collision.getWall());
            if(collision.isTrueCollision()){
                handleWallBounce(particleA, collision.getWall());
            }else{
                logger.info("False Collision between particle {} to vertical wall at y={} and x={}", particleA.getId(), particleA.getBallPositionY(), particleA.getBallPositionX());
            }
        }else{
            handleParticleCollision(particleA, particles.get(collision.getParticleB()));
        }

        calculateParticleCollisions(particleA);
        if (collision.getParticleB() != -1 && collision.getParticleB() < particlesCount) {
            Particle particleB = particles.get(collision.getParticleB());
            calculateParticleCollisions(particleB);
        }
    }

    /**
     * Calculates all initial collisions for all particles
     */
    private void calculateInitialCollisions() {
        for (Particle p1 : particles) {
            calculateParticleCollisions(p1);
        }
    }

    /**
     * Calculates collisions for said particle
     * @param particle particle
     */
    private void calculateParticleCollisions(Particle particle) {
        cleanCollisions(particle);
        // Collisions with walls
        particle.addCollision(timeToVerticalWall(particle));
        particle.addCollision(timeToHorizontalWall(particle));

        // Collisions with other particles
        for (Particle other : particles) {
            if (other.getId() == particle.getId()) continue; // Skip self-collision
            double dx = other.getBallPositionX() - particle.getBallPositionX();
            double dy = other.getBallPositionY() - particle.getBallPositionY();
            double distance = Math.sqrt(dx * dx + dy * dy);
            if (distance < 2 * ballRadius) {
                double time = timeToCollision(particle, other);
                logger.debug("Predicted collision between particle {} and {} in time {}", particle.getId(), other.getId(), time);
                if (time != Double.POSITIVE_INFINITY && time >= 0) {
                    Collision collision = new Collision(time, particle.getId(), other.getId(), null);
                    particle.addCollision(collision);
                    other.addCollision(collision);
                }
            }
        }
    }



    private double timeToCollision(Particle a, Particle b) {
        double dx  = b.getBallPositionX() - a.getBallPositionX();
        double dy  = b.getBallPositionY() - a.getBallPositionY();
        double dvx = b.getBallVelocityX() - a.getBallVelocityX();
        double dvy = b.getBallVelocityY() - a.getBallVelocityY();

        double dvdr = dx*dvx + dy*dvy;                // r·v
        double dvdv = dvx*dvx + dvy*dvy;              // v·v
        double drdr = dx*dx + dy*dy;                  // r·r
        double sigma = a.getBallRadius() + b.getBallRadius(); // R1+R2 (no asumas 2*ballRadius si pueden diferir)

        // logger.debug("dx: {}", dx);
        // logger.debug("dy: {}", dy);
        // logger.debug("dvx: {}", dvx);
        // logger.debug("dvy: {}", dvy);
        // logger.debug("dvdr (r·v): {}", dvdr);
        // logger.debug("dvdv (v·v): {}", dvdv);
        // logger.debug("drdr (r·r): {}", drdr);
        // logger.debug("sigma (R1+R2): {}", sigma);



        // No se aproximan (o casi tangencial)
        if (dvdr >= -EPS) return Double.POSITIVE_INFINITY;

        // Sin velocidad relativa
        if (dvdv <= EPS) return Double.POSITIVE_INFINITY;

        double d = dvdr*dvdr - dvdv*(drdr - sigma*sigma);
        if (d < 0) return Double.POSITIVE_INFINITY;

        double sqrtD = Math.sqrt(d);

        // logger.debug("Discriminant: {} and sqrtD: {}", d, sqrtD);
        double t1 = (-dvdr - sqrtD) / dvdv;
        double t2 = (-dvdr + sqrtD) / dvdv;

        double t = Double.POSITIVE_INFINITY;
        if (t1 >= 0) t = t1;
        if (t2 >= 0 && t2 < t) t = t2;

        logger.debug("Time to collision between particle {} and {}: {}", a.getId(), b.getId(), t);

        // Si ambas negativas, no hay choque futuro
        return t;
    }


    /**
     * Function that calculates the time it takes for a particle to collide with Horizontal walls.
     * If the particle is in the edges of the opening (0.0885 or 0.0915) then the comparison
     * depends on where the particle is moving, if its from A to B or from B to A
     *
     * @param particle describes the particle to study
     * @return the new Collision item set to the corresponding wall
     */
    private Collision timeToHorizontalWall(Particle particle) {
        double y = particle.getBallPositionY();
        double x = particle.getBallPositionX();
        boolean inBoxA = x < width - ballRadius;
        boolean inBoxB = x > width + ballRadius;
        double finalPosition;

        if (particle.getBallVelocityY() > 0) {
            double topWall;
            if(inBoxA){
                topWall = heightFirstBox;
            } else if (inBoxB) {
                topWall = topWallB;
            }else{
                topWall = particle.getBallVelocityX() > 0 ? topWallB : heightFirstBox;
            }
            finalPosition = topWall - ballRadius;
        } else {
            double bottomWall;
            if(inBoxA){
                bottomWall = 0;
            } else if (inBoxB) {
                bottomWall = bottomWallB;
            }else{
                bottomWall = particle.getBallVelocityX() > 0 ? bottomWallB : 0;
            }
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
     * @param particle describes the particle to study
     * @return the new Collision item set to the corresponding wall
     */
    private Collision timeToVerticalWall(Particle particle) {
        double finalPosition;
        double x = particle.getBallPositionX();
        boolean inBoxA = x < width - ballRadius;
        boolean inBoxB = x > width + ballRadius;

        if (particle.getBallVelocityX() > 0) {
            double rightWall = inBoxA ? width : width * 2;
            finalPosition = rightWall - ballRadius;
        } else {
            double leftWall = inBoxB ? width : 0;
            finalPosition = leftWall + ballRadius;
        }

        double time = timeToPosition(x, finalPosition, particle.getBallVelocityX());
        boolean atOpeningHeight = atOpeningHeight(particle.getBallPositionY(), time, particle.getBallVelocityY());
        boolean isFalse = atOpeningHeight && ((particle.getBallVelocityX() > 0 && inBoxA) || (particle.getBallVelocityX() < 0 && !inBoxA));
        logger.debug("----- Vertical Wall Collision ------");
        logger.debug("particle {}", particle.getId());
        logger.debug("velocity {}", particle.getBallVelocityX());
        logger.debug("initial position {}", particle.getBallPositionX());
        logger.debug("final position {}", finalPosition);
        logger.debug("time {}",time);
        logger.debug("COLLISION IS {}", isFalse ? "FALSE" : "TRUE");
        logger.debug("-----------");


        if (isFalse) {
            return new Collision(time, particle.getId(), Wall.VERTICAL, false);
        }

        return new Collision(time, particle.getId(), Wall.VERTICAL);
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

    /* ----------------------- Helper Functions ----------------------- */

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

    private void saveSimulationState(String filePath, boolean printHeaders, List<Particle> particlesToSave) {
        // Logic to save the simulation state to a file
        Locale.setDefault(Locale.US);
        try (PrintWriter writer = new PrintWriter(new FileWriter(filePath, !printHeaders))) { // append mode if not printing headers
            // Write headers
            if (printHeaders) {
                writer.println("N: " + particlesCount);
                writer.println("L: " + heightSecondBox);
            }
            writer.printf("%g;", totalTime); // fix with pressure

            for (Particle p : particlesToSave) {
                writer.printf("%d;", p.getId());
            }

            writer.printf("\n");

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

}