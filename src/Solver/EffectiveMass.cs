using System.Numerics;

namespace JomolithSolver.Solver;

public struct EffectiveMass(in Vector3 linear, in Vector3 angular)
{
    public Vector3 Linear = linear;
    public Vector3 Angular = angular;

    public EffectiveMass() : this(Vector3.Zero, Vector3.Zero)
    {
    }

    public void Reset()
    {
        Linear = Vector3.Zero;
        Angular = Vector3.Zero;
    }

    public void ApplyMultiplier(float m)
    {
        Linear *= m;
        Angular *= m;
    }
}

public struct EffectiveMassPair(in EffectiveMass a, in EffectiveMass b)
{
    public EffectiveMass A = a;
    public EffectiveMass B = b;

    public void Reset()
    {
        A.Reset();
        B.Reset();
    }

    public void ApplyMultipliers(float mA, float mB)
    {
        A.ApplyMultiplier(mA);
        B.ApplyMultiplier(mB);
    }
}
