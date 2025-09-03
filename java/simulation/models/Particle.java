
package models;

import java.util.PriorityQueue;

public class Particle {

    private final double ballMass = 1; // in kg
    private final double ballRadius = 0.0015; // meters

    private final int id;
    
    private double ballPositionX = 0; // in meters
    private double ballPositionY = 0; // in meters

    private double ballVelocityX = 0; // in m/s
    private double ballVelocityY = 0; // in m/s

    private final PriorityQueue<Collision> collisions = new PriorityQueue<>();

    public Particle(int id, double positionX, double positionY, double velocityX, double velocityY) {
        this.id = id;
        this.ballPositionX = positionX;
        this.ballPositionY = positionY;
        this.ballVelocityX = velocityX;
        this.ballVelocityY = velocityY;
    }

    public int getId(){
        return id;
    }

    public void setBallPosition(double ballPositionX, double ballPositionY) {
        this.ballPositionX = ballPositionX;
        this.ballPositionY = ballPositionY;
    }

    public void setBallVelocity(double ballVelocityX, double ballVelocityY) {
        this.ballVelocityX = ballVelocityX;
        this.ballVelocityY = ballVelocityY;
    }

    public void addCollision(Collision collision) {
        collisions.add(collision);
    }

    public void clearCollisions() {
        collisions.clear();
    }

    public Collision getNextCollision() {
        return collisions.poll();
    }

    public boolean hasCollisions() {
        return !collisions.isEmpty();
    }

    public void removeCollisionWithParticle(int id){
        collisions.removeIf(collision -> collision.getParticleB().getId() == id);
    }

    public PriorityQueue<Collision> getCollisions() {
        return collisions;
    }

    public double getBallMass() {
        return ballMass;
    }

    public double getBallRadius() {
        return ballRadius;
    }

    public double getBallVelocityX() {
        return ballVelocityX;
    }

    public double getBallVelocityY() {
        return ballVelocityY;
    }

    public double getBallPositionX() {
        return ballPositionX;
    }

    public double getBallPositionY() {
        return ballPositionY;
    }

}
