package org.models;


import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SimulationTest {

    private Simulation sim;
    private Method timeToVerticalWall;
    private Method timeToHorizontalWall;
    private Method timeToCollision;
    private Method handleParticleCollision;

    @BeforeEach
    void setUp() throws NoSuchMethodException{
        sim = new Simulation(0.06, 1);
        timeToVerticalWall = Simulation.class.getDeclaredMethod("timeToVerticalWall", Particle.class);
        timeToHorizontalWall = Simulation.class.getDeclaredMethod("timeToHorizontalWall", Particle.class);
        timeToCollision = Simulation.class.getDeclaredMethod("timeToCollision", Particle.class, Particle.class);
        handleParticleCollision = Simulation.class.getDeclaredMethod("handleParticleCollision", Particle.class, Particle.class);

        // Allow access to the private methods
        timeToVerticalWall.setAccessible(true);
        timeToHorizontalWall.setAccessible(true);
        timeToCollision.setAccessible(true);
        handleParticleCollision.setAccessible(true);
    }

    /* --------------------- Vertical Wall Collision Cases --------------------- */
    @Test
    void testVerticalWallCollisionWithNegativeVxVy() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.04, 0.08, -0.005, -0.0086);
        double expectedTime = 7.7;

        // When we call the private method
        Collision collision = (Collision) timeToVerticalWall.invoke(sim, particle);

        // Then the collision time and wall type should be correct
        assertNotNull(collision);
        assertEquals(expectedTime, collision.getTime(), 1e-5);
        assertEquals(CollisionType.VERTICAL, collision.getCollisionType());
    }

    @Test
    void testVerticalWallCollisionFalseCollision() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.08, 0.045, 0.1, 0);

        Collision collision = (Collision) timeToVerticalWall.invoke(sim, particle);

        assertNotNull(collision);
        assertEquals(CollisionType.VERTICAL, collision.getCollisionType());
        assertFalse(collision.isTrueCollision());
        assertEquals(0.085, collision.getTime(), 1e-5);
    }



    /* --------------------- Horizontal Wall Collision Cases --------------------- */
    @Test
    void testHorizontalWallCollisionWithNegativeVxVy() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.04, 0.08, -0.005, -0.0086);;
        double expectedTime = 9.127906977;

        // When we call the private method
        Collision collision = (Collision) timeToHorizontalWall.invoke(sim, particle);

        // Then the collision time and wall type should be correct
        assertNotNull(collision);
        assertEquals(expectedTime, collision.getTime(), 1e-5);
        assertEquals(CollisionType.HORIZONTAL, collision.getCollisionType());
    }

    @Test
    void testHorizontalWallCollisionFalseCollisionForParticleFromBoxAToBoxB() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.0885, 0.045, 0.1*Math.cos(Math.PI / 6),  0.1*Math.sin(Math.PI / 6));

        Collision collision = (Collision) timeToHorizontalWall.invoke(sim, particle);

        assertNotNull(collision);
        assertEquals(CollisionType.HORIZONTAL, collision.getCollisionType());
        assertTrue(collision.isTrueCollision());
        assertEquals(0.57, collision.getTime(), 1e-5);
    }


    /* --------------------- Particle Wall Collision Cases --------------------- */

    @Test
    void testTimeOfCollidingParticlesAgainstEachOther() throws InvocationTargetException, IllegalAccessException {
        Particle p1 = new Particle(0, 0.01, 0.01, 0.01, 0);
        Particle p2 = new Particle(2, 0.02, 0.01, -0.01, 0);

        Double collisionTime = (Double) timeToCollision.invoke(sim, p1, p2);

        assertNotNull(collisionTime);
        assertEquals(0.35, collisionTime, 1e-5);
        assertEquals(0.0001, Math.pow(p1.getBallVelocityX(), 2) + Math.pow(p1.getBallVelocityY(), 2), 1e-5);
    }

    @Test
    void testCollidingParticlesAgainstEachOtherVelocityAfter() throws InvocationTargetException, IllegalAccessException {
        Particle p1 = new Particle(0, 0.01-0.0015, 0.01, 0.01, 0);
        Particle p2 = new Particle(2, 0.01+0.0015, 0.01, -0.01, 0);

        handleParticleCollision.invoke(sim,p1,p2);

        assertEquals(-0.01, p1.getBallVelocityX());
        assertEquals(0.01, p2.getBallVelocityX());
    }

    @Test
    void NoMansLandEdgeCase_FromAtoB_ShouldTargetBoxATopWall() throws InvocationTargetException, IllegalAccessException {
        double width = 0.09;
        double ballRadius = 0.0015;

        Particle particle = new Particle(0, width + (0.5 * ballRadius), 0.4, -0.01, 0.01);

        double expectedTime = 3.85;
        Collision collision = (Collision) timeToHorizontalWall.invoke(sim, particle);
        assertNotNull(collision);
        assertEquals(CollisionType.HORIZONTAL, collision.getCollisionType());
        assertTrue(collision.isTrueCollision());
        assertEquals(expectedTime, collision.getTime(), 1e-5);
    }
}
