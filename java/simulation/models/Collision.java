package models;

public class Collision implements Comparable<Collision> {
    private final double time; // Time until collision occurs
    private final Particle particleA; // Main Particle
    private final Particle particleB; // Collision with
    private final Wall wall;
    private final boolean fake;

    public Collision(double time, Particle particleA, Particle particleB, Wall wall, boolean fake) {
        this.time = time;
        this.particleA = particleA;
        this.particleB = particleB;
        this.wall = wall;
        this.fake = fake;
    }

    public double getTime() {
        return time;
    }

    public Particle getParticleA() {
        return particleA;
    }

    public Particle getParticleB() {
        return particleB;
    }

    public Wall getWall() {
        return wall;
    }

    public boolean collisionWithWall(){
        return wall != null;
    }

    @Override
    public int compareTo(Collision other) {
        return Double.compare(this.time, other.time);
    }
}
