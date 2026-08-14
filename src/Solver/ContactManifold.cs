using System;
using System.Collections.Generic;
using System.Numerics;
using JomolithSolver.Solver.Constraints;

namespace JomolithSolver.Solver;

public struct ContactPoint
{
    public Vector3 PositionOnA;
    public Vector3 Normal;
    public float Depth;
    public float Friction;
    public float Restitution;
}

public class ContactManifold(Body a, Body b)
{
    private static long globalConstraintUid;

    public Body RootA { get; init; } = a;
    public Body RootB { get; init; } = b;
    public List<ConstraintCollision> Collisions { get; } = new();

    public void Update(IReadOnlyList<ContactPoint> freshContacts)
    {
        var clean = new List<ContactPoint>(freshContacts.Count);

        foreach (var point in freshContacts)
            if (Vector3.Dot(point.Normal, point.Normal) > 0.8f)
                clean.Add(point);

        if (clean.Count == 0)
        {
            Collisions.Clear();
            return;
        }

        var newIdx = 0;
        var cachedIdx = 0;

        for (; cachedIdx < Collisions.Count && newIdx < clean.Count; cachedIdx++, newIdx++)
        {
            var point = clean[newIdx];
            var c = Collisions[cachedIdx];
            c.Normal = point.Normal;
            c.PointA = point.PositionOnA;
            c.Depth = point.Depth;
            c.Friction = point.Friction;
            c.Restitution = point.Restitution;
        }

        for (; newIdx < clean.Count; newIdx++)
        {
            var point = clean[newIdx];
            var c = new ConstraintCollision(RootA, RootB);
            c.SetUID(globalConstraintUid++);
            c.Normal = point.Normal;
            c.PointA = point.PositionOnA;
            c.Depth = point.Depth;
            c.Friction = point.Friction;
            c.Restitution = point.Restitution;
            Collisions.Add(c);
        }

        if (clean.Count < Collisions.Count)
            Collisions.RemoveRange(clean.Count, Collisions.Count - clean.Count);

        Console.WriteLine($"[manifold] cached={Collisions.Count} fresh={clean.Count}");
    }
}
