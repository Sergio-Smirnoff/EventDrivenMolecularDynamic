package org.models;


import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import static org.junit.jupiter.api.Assertions.*;

public class SimulationTest {

    private Simulation sim;
    private Method verticalWallCollisionMethod;
    private Method horizontalWallCollisionMethod;

    @BeforeEach
    void setUp() throws NoSuchMethodException{
        sim = new Simulation(0.09, 1);
        verticalWallCollisionMethod = Simulation.class.getDeclaredMethod("VerticalWallCollision", Particle.class);
        horizontalWallCollisionMethod = Simulation.class.getDeclaredMethod("HorizontalWallCollision", Particle.class);

        // Allow access to the private methods
        verticalWallCollisionMethod.setAccessible(true);
        horizontalWallCollisionMethod.setAccessible(true);
    }

    /* --------------------- Vertical Wall Collision Cases --------------------- */
    @Test
    void testVerticalWallCollisionWithNegativeVxVy() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.04, 0.08, -0.005, -0.0086);
        double expectedTime = 7.7;

        // When we call the private method
        Collision collision = (Collision) verticalWallCollisionMethod.invoke(sim, particle);

        // Then the collision time and wall type should be correct
        assertNotNull(collision);
        assertEquals(expectedTime, collision.getTime(), 1e-5);
        assertEquals(Wall.VERTICAL, collision.getWall());
    }

    /* --------------------- Horizontal Wall Collision Cases --------------------- */
    @Test
    void testHorizontalWallCollisionWithNegativeVxVy() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.04, 0.08, -0.005, -0.0086);;
        double expectedTime = 9.127906977;

        // When we call the private method
        Collision collision = (Collision) horizontalWallCollisionMethod.invoke(sim, particle);

        // Then the collision time and wall type should be correct
        assertNotNull(collision);
        assertEquals(expectedTime, collision.getTime(), 1e-5);
        assertEquals(Wall.HORIZONTAL, collision.getWall());
    }
}
