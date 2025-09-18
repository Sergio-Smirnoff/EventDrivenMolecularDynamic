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

import static java.lang.System.exit;


public class Simulation {

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

    /** ============ FOR TESTING PURPOSES ============ **/
    public void manualAddParticle(Particle particle) {
        particles.add(particle);
    }

    /* ----------------------- Main Simulation Loop Functions ----------------------- */

    public void runSimulation(int maxIterations, String filepath){
        logger.info("Starting Simulation with {}", particlesCount);
        initializeSystem();
        saveSimulationState(filepath, true, new ArrayList<>()); // save initial state
        calculateInitialCollisions();

        List<Particle> collisionedParticles = new ArrayList<>();

        for (int collisionCount = 1; collisionCount <= maxIterations; collisionCount++) {
            logger.info("");
            logger.info("");
            logger.info("***********************");
            logger.info("Starting New Collision Analysis");
            logger.info("------- Collision number: {} -------", collisionCount);

            // 1. Find the very next event in the entire system.
            Collision nextEvent = findNextEvent();

            if (nextEvent == null || nextEvent.getTime() == Double.POSITIVE_INFINITY) {
                logger.warn("No more collisions can be predicted. Ending simulation.");
                break;
            }

            logger.info("Info About Collision:");
            logger.info(" ");
            logger.info("colliding time {}", nextEvent.getTime());
            logger.info("colliding particle {}", nextEvent.getParticleA());
            if (nextEvent.getParticleB() != -1)
                logger.info("colliding with particle {}", nextEvent.getParticleB());
            else if (nextEvent.getCollisionType() != CollisionType.PARTICLE)
                logger.info("colliding with wall {}", nextEvent.getCollisionType());
            logger.info("collision {}", nextEvent.isTrueCollision() ? "true":"false");
            logger.info(" ");

            // 3. Advance the system to the time of that event.
            if(!advanceSystem(nextEvent.getTime()))
                return;

            // 4. Resolve the collision and re-predict futures for involved particles.
            resolveCollision(nextEvent);

            // 5. Save the state of the system.
            if ( nextEvent.getParticleA() >= 0 && nextEvent.getParticleA() < particlesCount + 2 ) {
                collisionedParticles.add(particles.get(nextEvent.getParticleA()));
            }
            if ( nextEvent.getParticleB() >= 0 && nextEvent.getParticleB() < particlesCount + 2 ) {
                collisionedParticles.add(particles.get(nextEvent.getParticleB()));
            }

            saveSimulationState(filepath, false, collisionedParticles);
            collisionedParticles.clear();
            logger.info("***********************");
            logger.info("");
            logger.info("");
        }
    }



    /**
     * Finds the next event that will occur
     *
     * @return collision that will occur next
     */
    private Collision findNextEvent() {
        Collision nextEvent = null;
        for (int i = 2; i < particlesCount + 2; i++) {
            Particle p = particles.get(i);
            if (!p.hasCollisions()) continue;
            Collision earliestForP = p.getNextCollision();
            if (nextEvent == null || earliestForP.getTime() < nextEvent.getTime()) {

                logger.debug("Found new next event for particle {}: time {}", p.getId(), earliestForP.getTime());
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

        logger.debug("Advancing all particles positions by {} seconds. Total time: {}s", timeSkip, totalTime);
        logger.debug("");

        for (int i = 2; i < particlesCount + 2; i++) {
            Particle p = particles.get(i);
            double newX = p.getBallPositionX() + p.getBallVelocityX() * timeSkip;
            double newY = p.getBallPositionY() + p.getBallVelocityY() * timeSkip;
            logger.debug("New X: {} New Y: {} for particle {}", newX, newY, p.getId());
            p.setBallPosition(newX, newY);
            for (Collision c : p.getCollisions()) {
                double newTime = c.getTime() - timeSkip;
                if (newTime < 0 && newTime > -EPS) {
                    newTime = 0.0;
                } else if (newTime < -EPS) {
                    logger.warn("Collision time became significantly negative ({}) for collision {} after advancing. Clamping to +Inf", newTime, c);
                    newTime = Double.POSITIVE_INFINITY; // o elimina la colisión  // TODO checkear si es mejor eliminarla
                }
                c.setTime(newTime);
            }

        }

        totalTime += timeSkip;
        logger.debug("");
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
            if (collision.getCollisionType() == CollisionType.PARTICLE) {
                Particle particleB = particles.get(collision.getParticleB());
                particleB.getCollisions().removeIf((col) -> col.getParticleB() == collidingParticle.getId());
            }
        }
        collidingParticle.clearCollisions();
    }



    /**
     * Given a particle that collides with a wall, or to the invisible wall, corrects
     * the value to avoid things like y = 0.00150000000000002 or y = 0.001499999999999999999996
     * If the particle happens to be in the edges of the opening (0.0885 or 0.0915) then
     * it leaves it as it is, unless the collision is with a horizontal wall, then it will snap it
     *
     * @param p particle which collides
     * @param wall wall to which the particle collides
     */
    private void snapToWall(Particle p, CollisionType wall){
        logger.debug("Inside snap function to check if particle {} must snap to {} wall", p.getId(), wall);
        logger.debug("where x={} and y={}", p.getBallPositionX(), p.getBallPositionY());
        double x = p.getBallPositionX();
        double y = p.getBallPositionY();
        boolean inBoxA = x < width - ballRadius || (x > width - ballRadius && Math.abs(width-ballRadius-x) < EPS);
        boolean inBoxB = x > width + ballRadius || (x < width + ballRadius && Math.abs(width+ballRadius-x) < EPS);
        if(wall == CollisionType.VERTICAL || wall == CollisionType.VERTEX){
            double newX = p.getBallPositionX();
            if(p.getBallVelocityX()>0){
                if(inBoxA){
                    newX = width - ballRadius;
                    logger.info("snapped particle to vertical wall at x: {} y: {}", newX, p.getBallPositionY());
                } else if (inBoxB) {
                    newX = 2 * width - ballRadius;
                    logger.info("snapped particle to vertical wall at x: {} y: {}", newX, p.getBallPositionY());
                }else{
                    logger.info("left particle {} the same at x: {} y: {}", p.getId(), p.getBallPositionX(), p.getBallPositionY());
                }
                // x == 0.0885 or 0.0915
            }else{
                if(inBoxA){
                    newX = ballRadius;
                    logger.info("snapped particle to vertical wall at x: {} y: {}", newX, p.getBallPositionY());
                } else if (inBoxB) {
                    newX = width + ballRadius;
                    logger.info("snapped particle to vertical wall at x: {} y: {}", newX, p.getBallPositionY());
                }
                else {
                    logger.info("left particle {} the same at x: {} y: {}", p.getId(), p.getBallPositionX(), p.getBallPositionY());
                }
            }
            p.setBallPosition(newX, p.getBallPositionY());
        }
        if(wall == CollisionType.HORIZONTAL || wall == CollisionType.VERTEX){
            double newY = p.getBallPositionY();
            if(p.getBallVelocityY() > 0){
                if(inBoxA){
                    newY = heightFirstBox - ballRadius;
                    logger.info("snapped particle to horizontal wall at x: {} y: {}", p.getBallPositionX(), newY);
                } else {
                    newY = heightSecondBox == 0.03 ? 0.0585: topWallB - ballRadius;
                    logger.info("snapped particle to horizontal wall at x: {} y: {}", p.getBallPositionX(), newY);
                }
            }else{
                if(inBoxA){
                    newY = ballRadius;
                    logger.info("snapped particle to horizontal wall at x: {} y: {}", p.getBallPositionX(), newY);
                } else {
                    newY = bottomWallB + ballRadius;
                    logger.info("snapped particle {} to horizontal wall at x: {} y: {}", p.getId(), p.getBallPositionX(), newY);
                }
            }
            p.setBallPosition(p.getBallPositionX(), newY);

        }
        logger.debug("");
    }

    /**
     * Updates velocities of colliding particle depending on wall collision
     * @param p colliding particle
     * @param w colliding wall
     */
    private void handleWallBounce(Particle p, CollisionType w){
        double vx = p.getBallVelocityX();
        double vy = p.getBallVelocityY();
        logger.debug("handling {} wall bounce for particle {}", w, p.getId());
        if (w.equals(CollisionType.VERTICAL)) {
            p.setBallVelocity(-vx, vy);
        } else if (w.equals(CollisionType.HORIZONTAL)) {
            p.setBallVelocity(vx, -vy);
        }else{
            // depende como venga la partícula hacia el vértice ?
            p.setBallVelocity(-p.getBallVelocityX(), -p.getBallVelocityY());
        }
    }

        // check
        private void handleParticleCollision(Particle a, Particle b) {

            double dx = b.getBallPositionX() - a.getBallPositionX();
            double dy = b.getBallPositionY() - a.getBallPositionY();
            double dist = Math.hypot(dx, dy);
            logger.info("distance between particles {} and {}: {}", a.getId(), b.getId(), dist);
            if (dist == 0.0) return;

            double nx = dx / dist, ny = dy / dist;
            double dvx = b.getBallVelocityX() - a.getBallVelocityX();
            double dvy = b.getBallVelocityY() - a.getBallVelocityY();
            double vn = dvx * nx + dvy * ny;


            if (vn > -EPS) {
                return;
            }

            double mi = a.getBallMass();
            double mj = b.getBallMass();
            double J;

            if (mi == Double.POSITIVE_INFINITY || mj == Double.POSITIVE_INFINITY) {
                // Collision with an immovable object (vertex)
                if (mi == Double.POSITIVE_INFINITY) { // Particle 'a' is the wall
                    J = (2 * mj) * vn;
                } else { // Particle 'b' is the wall
                    J = (2 * mi) * vn;
                }
            } else {
                // Standard two-particle collision
                J = ((2 * mj * mi) / (mi + mj)) * vn;
            }

            logger.info("previous velocities for colliding particles {} against {}", a.getId(), a.getId());
            logger.info("particle {}: vx {}    vy {}", a.getId(), a.getBallVelocityX(), a.getBallVelocityY());
            logger.info("particle {}: vx {}    vy {}", b.getId(), b.getBallVelocityX(), b.getBallVelocityY());

            // actualizar velocidades: v' = v ± (J/m) * n
            double newVxForA = a.getBallVelocityX() + (J / mi) * nx;
            double newVyForA = a.getBallVelocityY() + (J / mi) * ny;
            double newVxForB = b.getBallVelocityX() - (J / mj) * nx;
            double newVyForB = b.getBallVelocityY() - (J / mj) * ny;

            logger.info("new velocities for colliding particles {} against {}", a.getId(), a.getId());
            logger.info("particle {}: vx {}    vy {}", a.getId(), newVxForA, newVyForA);
            logger.info("particle {}: vx {}    vy {}", b.getId(), newVxForB, newVyForB);
            a.setBallVelocity( newVxForA, newVyForA);
            b.setBallVelocity(newVxForB, newVyForB);

            logger.info("Colliding particle {} with particle {}", a.getId(), b.getId());
        }

    /**
     * Resolves occurring collision. Analyses if wall collision or particle collision occur.
     * If particle collides with invisible wall, it only recalculates collision since particle
     * changes environment.
     * @param collision occurring collision
     */
    private void resolveCollision(Collision collision) {
        logger.debug("Resolving collision: type {}", collision.getCollisionType());
        logger.debug("");
        Particle particleA = particles.get(collision.getParticleA());
        if(collision.getCollisionType() != CollisionType.PARTICLE){
            snapToWall(particleA, collision.getCollisionType());
            if(collision.isTrueCollision()){
                handleWallBounce(particleA, collision.getCollisionType());
            }else{
                logger.debug("particle does not bounce off wall");
                logger.info("False Collision between particle {} to vertical wall at y={} and x={}", particleA.getId(), particleA.getBallPositionY(), particleA.getBallPositionX());
            }
        }else{
            logger.info("HANDLING PARTICLE COLLISION BETWEEN {} AND {}", particleA.getId(), collision.getParticleB());
            handleParticleCollision(particleA, particles.get(collision.getParticleB()));
        }

        calculateParticleCollisions(particleA);
        if (collision.getParticleB() > 1 && collision.getParticleB() < particlesCount + 2) {
            Particle particleB = particles.get(collision.getParticleB());
            calculateParticleCollisions(particleB);
        }
        logger.debug("");
    }

    /**
     * Calculates all initial collisions for all particles
     */
    private void calculateInitialCollisions() {
        logger.debug("");
        logger.debug("Calling Initial Collisions");
        for (int i = 2; i < particlesCount + 2; i++) {
            Particle particle = particles.get(i);
            calculateParticleCollisions(particle);
        }
        logger.debug("");
    }

    /**
     * Calculates collisions for said particle
     * @param particle particle
     */
    private void calculateParticleCollisions(Particle particle) {
        cleanCollisions(particle);
        logger.debug("");
        logger.info("CALCULATING COLLISIONS FOR PARTICLE {}", particle.getId());
        // Collisions with walls
        Collision collisionVertical = timeToVerticalWall(particle);
        Collision collisionHorizontal = timeToHorizontalWall(particle);
        if(Math.abs(collisionVertical.getTime() - collisionHorizontal.getTime()) > EPS){
            particle.addCollision(collisionVertical);
            particle.addCollision(collisionHorizontal);
        }else{
            collisionVertical.setCollisionType(CollisionType.VERTEX);
            particle.addCollision(collisionVertical);
        }

        // Collisions with other particles
        for (Particle other : particles) {
            if (other.getId() == particle.getId()) continue; // Skip self-collision
            double dx = other.getBallPositionX() - particle.getBallPositionX();
            double dy = other.getBallPositionY() - particle.getBallPositionY();
            double distance = Math.sqrt(dx * dx + dy * dy);
                double time = timeToCollision(particle, other);
                if (time != Double.POSITIVE_INFINITY && time >= 0) {
                    Collision collision = new Collision(time, particle.getId(), other.getId(), CollisionType.PARTICLE);
                    particle.addCollision(collision);
                    if(other.getId() > 1){
                        Collision secondColl = new Collision(time, other.getId(), particle.getId(), CollisionType.PARTICLE);
                        other.addCollision(secondColl);
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
        if (drdr <= sigma*sigma + EPS) {
            // ya están tangentes/solapadas -> choque inmediato
            return 0.0;
        }


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
        double t1 = (-dvdr - sqrtD) / dvdv;
        double t2 = (-dvdr + sqrtD) / dvdv;

        // Elegir la solución válida en el futuro
        double t = Double.POSITIVE_INFINITY;
        if (t1 >= -EPS) t = Math.max(0.0, t1);
        if (t2 >= -EPS && t2 < t) t = Math.max(0.0, t2);

        // Si ambas raíces claramente negativas -> no choque futuro
        if (t == Double.POSITIVE_INFINITY) {
            logger.debug("No future collision between {} and {}", a.getId(), b.getId());
        }
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
        double x = particle.getBallPositionX();
        double y = particle.getBallPositionY();
        boolean inBoxA = x < width - ballRadius; // && !(y >= bottomWallB + ballRadius && y <= (heightSecondBox == 0.03 ? 0.0585: topWallB - ballRadius));
        boolean inBoxB = x > width + ballRadius;
        double finalPosition;

        if (particle.getBallVelocityY() > 0) {
            if(inBoxA || (x == width - ballRadius && !(y >= bottomWallB + ballRadius && y <= (heightSecondBox == 0.03 ? 0.0585: topWallB - ballRadius)))){
                finalPosition = heightFirstBox - ballRadius;
            } else if (inBoxB) {
                finalPosition = heightSecondBox == 0.03 ? 0.0585: topWallB - ballRadius;
            } else{
                logger.info("collision for no mans land for particle {} in x {} y {}", particle.getId(), x, y);
                if(particle.getBallVelocityX()>0){
                    finalPosition = heightSecondBox == 0.03 ? 0.0585: topWallB - ballRadius;
                }else{
                    // desplazo la partícula a la zona crítica -> es necesario?
                    double lookAheadTime = timeToPosition(y, topWallB, particle.getBallVelocityY());
                    double lookAheadX = x + particle.getBallVelocityX() * lookAheadTime;
                    if(lookAheadX + ballRadius < width){
                        finalPosition = heightFirstBox - ballRadius;
                    }else{
                        finalPosition = heightSecondBox == 0.03 ? 0.0585: topWallB - ballRadius;
                    }
                }
                //topWall = particle.getBallVelocityX() > 0 ? topWallB : heightFirstBox;
            }
        } else {
            double bottomWall;
            if(inBoxA || (x == width - ballRadius && !(y >= bottomWallB + ballRadius && y <= (heightSecondBox == 0.03 ? 0.0585: topWallB - ballRadius)))){
                bottomWall = 0;
            } else if (inBoxB) {
                bottomWall = bottomWallB;
            }else{
                logger.info("collision for no mans land for particle {} in x {} y {}", particle.getId(), x, y);
                if(particle.getBallVelocityX()>0){
                    bottomWall = bottomWallB;
                }else{
                    // desplazo la partícula a la zona crítica
                    double lookAheadTime = timeToPosition(y, bottomWallB, particle.getBallVelocityY());
                    double lookAheadX = x + particle.getBallVelocityX() * lookAheadTime;
                    if(lookAheadX + ballRadius < width){
                        bottomWall = 0;
                    }else{
                        bottomWall = bottomWallB;
                    }
                }
                //bottomWall = particle.getBallVelocityX() > 0 ? bottomWallB : 0;
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

        // for debugging
        if(time < 0){
            logger.debug("");
            logger.debug("NEGATIVE TIME {} CALCULATED", time);
            exit(2);
        }
        return new Collision(time, particle.getId(), CollisionType.HORIZONTAL);
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
        return yf >= bottomWallB + ballRadius && yf <= (heightSecondBox == 0.03 ? 0.0585: topWallB - ballRadius);
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

        // for debugging
        if(time < 0){
            logger.debug("");
            logger.debug("NEGATIVE TIME {} CALCULATED", time);
            exit(2);
        }


        if (isFalse) {
            return new Collision(time, particle.getId(), CollisionType.VERTICAL, false);
        }

        return new Collision(time, particle.getId(), CollisionType.VERTICAL);
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
            for (
                int i = 2; i < particlesCount + 2; i++
            ) {
                Particle particle = particles.get(i);
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


    private void addVertexParticles(){
        Particle topVertexParticle = new Particle(0, width, topWallB, 0, 0, 0, Double.POSITIVE_INFINITY);
        Particle bottomVertexParticles = new Particle(1, width, bottomWallB, 0, 0, 0, Double.POSITIVE_INFINITY);
        particles.add(topVertexParticle);
        particles.add(bottomVertexParticles);
    }


    private void initializeSystem() {
        addVertexParticles();
        for (int i = 2; i < particlesCount + 2; i++) {
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

                    if (distance <= 2 * ballRadius) {
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