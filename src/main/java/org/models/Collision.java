package org.models;

public class Collision implements Comparable<Collision> {
    private int id;
    private double time; // Time until collision occurs
    private int particleA = -1; // Main Particle id
    private int particleB = -1; // Collision with id
    private final Wall wall;

    public Collision(double time, int particleA, int particleB, Wall wall) {
        this.time = time;
        this.particleA = particleA;
        this.particleB = particleB;
        this.wall = wall;
    }

    public Collision(double time, int particleA, Wall wall) {
        this.time = time;
        this.particleA = particleA;
        this.wall = wall;
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

    public Wall getWall() {
        return wall;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public boolean collisionWithWall(){
        return wall != null;
    }

    @Override
    public int compareTo(Collision other) {
        return Double.compare(this.time, other.time);
    }
}
