using System;
using System.Numerics;
using Vector3 = System.Numerics.Vector3;

namespace JomolithSolver.Solver;

public readonly struct MassProperties(float mass, Matrix4x4 inertiaBody)
{
    public readonly float Mass = mass;
    public readonly Matrix4x4 InertiaBody = inertiaBody;

    public static MassProperties ForShape(Body.ShapeType shape, float density, Vector3 size)
    {
        return shape switch
        {
            Body.ShapeType.Box => Box(density, size),
            Body.ShapeType.Sphere => Sphere(density, size.X / 2f),
            Body.ShapeType.Cylinder => Cylinder(density, size.Y / 2f, size.X),
            Body.ShapeType.Wedge => Wedge(density, size),
            Body.ShapeType.CornerWedge => CornerWedge(density, size),
            _ => Box(density, size)
        };
    }

    public static float DensityOf(Body.ShapeType shape, float mass, Vector3 size)
    {
        return shape switch
        {
            Body.ShapeType.Box => mass / BoxVolume(size),
            Body.ShapeType.Sphere => mass / SphereVolume(size.X / 2f),
            Body.ShapeType.Cylinder => mass / CylinderVolume(size.X / 2f, size.Y),
            Body.ShapeType.Wedge => mass / (BoxVolume(size) / 2f),
            Body.ShapeType.CornerWedge => mass / (BoxVolume(size) / 3f),
            _ => mass / BoxVolume(size)
        };
    }

    private static MassProperties Box(float density, Vector3 size)
    {
        var mass = density * BoxVolume(size);

        var squaredSize = size * size;

        var inertiaBody = Matrix4x4.Identity;

        inertiaBody.M11 = mass / 12.0f * (squaredSize.Y + squaredSize.Z);
        inertiaBody.M22 = mass / 12.0f * (squaredSize.X + squaredSize.Z);
        inertiaBody.M33 = mass / 12.0f * (squaredSize.X + squaredSize.Y);

        return new MassProperties(mass, inertiaBody);
    }

    private static float BoxVolume(Vector3 size) => size.X * size.Y * size.Z;

    private static MassProperties Sphere(float density, float radius)
    {
        var mass = density * SphereVolume(radius);

        var I = 0.4f * mass * radius * radius;

        var inertiaBody = Matrix4x4.Identity;

        inertiaBody.M11 = I;
        inertiaBody.M22 = I;
        inertiaBody.M33 = I;

        return new MassProperties(mass, inertiaBody);
    }

    private static float SphereVolume(float radius) => 4 / 3.0f * float.Pi * radius * radius * radius;

    private static MassProperties Cylinder(float density, float radius, float length)
    {
        var mass = density * CylinderVolume(radius, length);

        var iAxis = 0.5f * mass * radius * radius;
        var iPerp = mass / 12.0f * (3.0f * radius * radius + length * length);

        var inertiaBody = Matrix4x4.Identity;

        inertiaBody.M11 = iAxis;
        inertiaBody.M22 = iPerp;
        inertiaBody.M33 = iPerp;

        return new MassProperties(mass, inertiaBody);
    }

    private static float CylinderVolume(float radius, float length) => float.Pi * radius * radius * length;

    // We just use the bounding box to calculate wedge properties. Yeah. That's right.
    private static MassProperties Wedge(float density, Vector3 size) => Box(density / 2.0f, size);

    private static MassProperties CornerWedge(float density, Vector3 size) => Box(density / 3.0f, size);
}
