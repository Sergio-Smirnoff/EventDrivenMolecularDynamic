package org.models;


import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import static org.junit.jupiter.api.Assertions.*;

public class SimulationTest {

    private Simulation sim;
    private Method timeToVerticalWall;
    private Method timeToHorizontalWall;

    @BeforeEach
    void setUp() throws NoSuchMethodException{
        sim = new Simulation(0.06, 1);
        timeToVerticalWall = Simulation.class.getDeclaredMethod("timeToVerticalWall", Particle.class);
        timeToHorizontalWall = Simulation.class.getDeclaredMethod("timeToHorizontalWall", Particle.class);

        // Allow access to the private methods
        timeToVerticalWall.setAccessible(true);
        timeToHorizontalWall.setAccessible(true);
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
        assertEquals(Wall.VERTICAL, collision.getWall());
    }

    @Test
    void testVerticalWallCollisionFalseCollision() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.08, 0.045, 0.1, 0);

        Collision collision = (Collision) timeToVerticalWall.invoke(sim, particle);

        assertNotNull(collision);
        assertEquals(Wall.VERTICAL, collision.getWall());
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
        assertEquals(Wall.HORIZONTAL, collision.getWall());
    }

    @Test
    void testHorizontalWallCollisionFalseCollisionForParticleFromBoxAToBoxB() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.0885, 0.045, 0.1*Math.cos(Math.PI / 6),  0.1*Math.sin(Math.PI / 6));

        Collision collision = (Collision) timeToHorizontalWall.invoke(sim, particle);

        assertNotNull(collision);
        assertEquals(Wall.HORIZONTAL, collision.getWall());
        assertTrue(collision.isTrueCollision());
        assertEquals(0.57, collision.getTime(), 1e-5);
    }
}
