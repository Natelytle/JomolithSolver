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

public struct VirtualDisplacementArray(int length)
{
    private readonly VirtualDisplacement[] data = new VirtualDisplacement[length];

    public void Reset()
    {
        for (var i = 0; i < data.Length; i++) data[i].Reset();
    }

    public ref VirtualDisplacement this[int i] => ref data[i];

    public int Size => data.Length;
}
