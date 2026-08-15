using System.Numerics;

namespace JomolithSolver.Solver;

public struct VirtualDisplacement(in Vector3 linear, in Vector3 angular)
{
    public Vector3 Linear = linear;
    public Vector3 Angular = angular;

    public VirtualDisplacement() : this(Vector3.Zero, Vector3.Zero)
    {
    }

    public void Reset()
    {
        Linear = Vector3.Zero;
        Angular = Vector3.Zero;
    }
}
