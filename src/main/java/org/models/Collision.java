package org.models;

public class Collision implements Comparable<Collision> {
    private int id;
    private double time; // Time until collision occurs
    private int particleA = -1; // Main Particle id
    private int particleB = -1; // Collision with id
    private CollisionType collisionType;
    private boolean isTrueCollision = true;

    public Collision(double time, int particleA, int particleB, CollisionType collisionType) {
        this.time = time;
        this.particleA = particleA;
        this.particleB = particleB;
        this.collisionType = collisionType;
    }

    public Collision(double time, int particleA, CollisionType collisionType, boolean isTrueCollision) {
        this.time = time;
        this.particleA = particleA;
        this.collisionType = collisionType;
        this.isTrueCollision = isTrueCollision;
    }

    public Collision(double time, int particleA, CollisionType collisionType) {
        this.time = time;
        this.particleA = particleA;
        this.collisionType = collisionType;
    }

    public int getId(){ return id; }

    public double getTime() {
        return time;
    }

    public int getParticleA() {
        return particleA;
    }

    public int getParticleB() {
        return particleB;
    }

    public boolean isTrueCollision() {
        return isTrueCollision;
    }

    public CollisionType getCollisionType() {
        return collisionType;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public void setCollisionType(CollisionType collisionType) {
        this.collisionType = collisionType;
    }

    public boolean collisionWithWall(){
        return collisionType == CollisionType.HORIZONTAL || collisionType == CollisionType.VERTICAL;
    }

    @Override
    public int compareTo(Collision other) {
        return Double.compare(this.time, other.time);
    }
}
