using System.Numerics;

namespace JomolithSolver.Solver;

public struct ConstraintJacobian(in Vector3 linear, in Vector3 angular)
{
    public Vector3 Linear = linear;
    public Vector3 Angular = angular;

    public ConstraintJacobian() : this(Vector3.Zero, Vector3.Zero)
    {
    }

    public void Reset()
    {
        Linear = Vector3.Zero;
        Angular = Vector3.Zero;
    }
}

public struct ConstraintJacobianPair
{
    public ConstraintJacobian A;
    public ConstraintJacobian B;

    public void Reset()
    {
        A.Reset();
        B.Reset();
    }

    public float Dot(in EffectiveMassPair em)
    {
        float r = 0;
        r += Vector3.Dot(A.Linear, em.A.Linear);
        r += Vector3.Dot(A.Angular, em.A.Angular);
        r += Vector3.Dot(B.Linear, em.B.Linear);
        r += Vector3.Dot(B.Angular, em.B.Angular);
        return r;
    }
}
