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
    private Method snapToWall;
    private Method resolveCollision;
    private Method advanceSystem;

    @BeforeEach
    void setUp() throws NoSuchMethodException {
        sim = new Simulation(0.03, 1);
        timeToVerticalWall = Simulation.class.getDeclaredMethod("timeToVerticalWall", Particle.class);
        timeToHorizontalWall = Simulation.class.getDeclaredMethod("timeToHorizontalWall", Particle.class);
        timeToCollision = Simulation.class.getDeclaredMethod("timeToCollision", Particle.class, Particle.class);
        handleParticleCollision = Simulation.class.getDeclaredMethod("handleParticleCollision", Particle.class, Particle.class);
        snapToWall = Simulation.class.getDeclaredMethod("snapToWall", Particle.class, CollisionType.class);
        resolveCollision = Simulation.class.getDeclaredMethod("resolveCollision", Collision.class);
        advanceSystem = Simulation.class.getDeclaredMethod("advanceSystem", double.class);

        // Allow access to the private methods
        timeToVerticalWall.setAccessible(true);
        timeToHorizontalWall.setAccessible(true);
        timeToCollision.setAccessible(true);
        snapToWall.setAccessible(true);
        resolveCollision.setAccessible(true);
        advanceSystem.setAccessible(true);
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
        Particle particle = new Particle(0, 0.04, 0.08, -0.005, -0.0086);
        ;
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
        Particle particle = new Particle(0, 0.0885, 0.045, 0.1 * Math.cos(Math.PI / 6), 0.1 * Math.sin(Math.PI / 6));

        Collision collision = (Collision) timeToHorizontalWall.invoke(sim, particle);

        assertNotNull(collision);
        assertEquals(CollisionType.HORIZONTAL, collision.getCollisionType());
        assertTrue(collision.isTrueCollision());
        assertEquals(0.57, collision.getTime(), 1e-5);
    }

    @Test
    void testHorizontalWallCollisionForBouncingParticleInVerticalWallAtLeftEdgeOfNoMansLand() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.0885, 0.0762433029960601, -0.007922913366164434, 0.006101429651504059);

        Collision collision = (Collision) timeToHorizontalWall.invoke(sim, particle);

        assertNotNull(collision);
        assertEquals(CollisionType.HORIZONTAL, collision.getCollisionType());
        assertTrue(collision.isTrueCollision());
        assertEquals(2.008823785, collision.getTime(), 1e-5);
    }

    @Test
    void testHorizontalWallCollisionForNormalSituation() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.05, 0.045, 0, -0.01);

        Collision collision = (Collision) timeToHorizontalWall.invoke(sim, particle);

        assertNotNull(collision);
        assertEquals(CollisionType.HORIZONTAL, collision.getCollisionType());
        assertTrue(collision.isTrueCollision());
        assertEquals(4.35, collision.getTime(), 1e-5);
    }

    /* --------------------- Particle Collision Cases --------------------- */

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

    /* --------------------- No Mans Land Collision Cases --------------------- */

    /**
     * por algún motivo calcula 2 veces la colisión contra la pared vertical para este caso, o sea
     * para estas condiciones iniciales (el tema está en que no hacía snap, ahora lo hace bien:ver sig test)
     */
    @Test
    void NoMansLandEdgeCase_FromBtoA_FirstFalseWallCollision_HorizontalWall() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.09201420123594184, 0.05573448365597751, -0.006246366176356435, 0.012022873060501297);

        double expectedTime = 0.23002125449598465;
        Collision collision = (Collision) timeToHorizontalWall.invoke(sim, particle);

        assertNotNull(collision);
        assertEquals(CollisionType.HORIZONTAL, collision.getCollisionType());
        assertTrue(collision.isTrueCollision());
        assertEquals(expectedTime, collision.getTime(), 1e-5);
    }

    @Test
    void NoMansLandEdgeCase_FromBtoA_FirstFalseWallCollision_VerticalWall() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.09201420123594184, 0.05573448365597751, -0.006246366176356435, 0.012022873060501297);

        double expectedTime = 0.08232005960332281;
        Collision collision = (Collision) timeToVerticalWall.invoke(sim, particle);

        assertNotNull(collision);
        assertEquals(CollisionType.VERTICAL, collision.getCollisionType());
        assertFalse(collision.isTrueCollision());
        assertEquals(expectedTime, collision.getTime(), 1e-5);
    }

    /** antes tenía dos colisiones falsas para 0.0915000000005 o algo así y 0.0915 en ese orden,
     * y para la primera no hacía snap por las condiciones de borde:
     * boolean inBoxA = p.getBallPositionX() < width - ballRadius - EPS;
     *         boolean inBoxB = p.getBallPositionX() > width + ballRadius + EPS;
     * Si le saco el EPR funciona bien
     * mentira, o no del todo funciona bien, martu del futuro sufre porque el caso que justo esté por muuuy poquito adentro en la no mans land
     * no lo snapea, tengo que volver a ponerlo, tengo que subrir el caso por ejemplo para algo que se mueve a la derecha
     * y x = 0.088500000000000005, que se ve no lo cubría parece ser no se porque
    */
    @Test
    void NoMansLandEdgeCase_FromBtoA_FirstFalseWallCollision_Snap() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.08850000000000006, 0.0289298554237422, 0.00908713110168076, -0.004174212301844052);
        //Collision collision = new Collision(0.0046339936623408205, particle.getId(), CollisionType.VERTICAL, false);

        snapToWall.invoke(sim, particle, CollisionType.VERTICAL);
        assertEquals(0.0885, particle.getBallPositionX());
    }

    /**
     * El bug está cuando se calculan nuevamente los choques, toma heightBoxA como pared
     * horizontal cuando no debería, sino que tendría qeu ser topWallB.
     * Cuando está en la no mans land, la condición a chequear debería ser:
     *                  if(particle.getBallVelocityX()>0){
     *                     topWall = topWallB;
     *                 }else{
     *                     topWall = x > width ? topWallB : heightFirstBox;
     *                 }
     * Con La que estaba que es solamente:
     * topWall = particle.getBallVelocityX() > 0 ? topWallB : heightFirstBox;
     */

    @Test
    void NoMansLandEdgeCase_FromBtoA_FirstFalseWallCollision_RecalcCollisions() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.0915, 0.05672420728292126, -0.006246366176356435, 0.012022873060501297);

        Collision collision = (Collision) timeToHorizontalWall.invoke(sim, particle);

        assertNotNull(collision);
        assertEquals(CollisionType.HORIZONTAL, collision.getCollisionType());
        assertTrue(collision.isTrueCollision());
        assertEquals(0.1477011949, collision.getTime(), 1e-5);
    }


    /**
     * Ahora el problema está en cuando se maneja esa colisión, porque como que se hace tp al vértice de arriba
     */

    @Test
    void NoMansLandEdgeCase_FromBtoA_ResolveBuggyCollision() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.0915, 0.05672420728292126, -0.006246366176356435, 0.012022873060501297);
        Collision collision = new Collision(0.1477011949, particle.getId(), CollisionType.HORIZONTAL);

        sim.manualAddParticle(particle);

        advanceSystem.invoke(sim, collision.getTime());

        // en realidad da 0.05850000000008832, pero se debería de corregir con el snap
        assertEquals(0.0585, particle.getBallPositionY(), 1e-5);

        // en realidad, justo ese número no se corrige con snap porque (0.09-0.03)/2 = 0.058499999999999996
        // para evitar eso pasaría a hardcodear el número de top y bottom wall
        resolveCollision.invoke(sim, collision);
    }

    /**
     * El problema con esta condición:
     * topWall = x > width ? topWallB : heightFirstBox;
     * es que si la partícula en su trayectoria no llega a rozar el borde,, yendo de B a A
     * y con su x > width, va a pensar que debe chocar con el bottomWallB y no es así. Cambiando la
     * condición por:
     *Math.abs(particle.getBallVelocityY() - bottomWallB) < EPS
     * debería de arreglarse, no se como testearlo
     *
     * ok le metí trigonometría, pero creo que no funca o no lo codee bien
     */

    @Test
    void NoMansLandEdgeCase_FromBtoA_EdgeCondition() throws InvocationTargetException, IllegalAccessException {
        Particle particle = new Particle(0, 0.08862316099045994, 0.037228108517198225, -1.3673032785457666E-4, -0.01844666770426321);

        Collision collision = (Collision) timeToHorizontalWall.invoke(sim, particle);

        assertNotNull(collision);
        assertEquals(CollisionType.HORIZONTAL, collision.getCollisionType());
        assertTrue(collision.isTrueCollision());
    }
}
