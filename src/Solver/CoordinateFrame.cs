using System.Numerics;
using JomolithSolver.Solver.Utils;

namespace JomolithSolver.Solver;

public record struct CoordinateFrame(in Matrix4x4 Rotation, in Vector3 Translation)
{
    // We're only using the top 3x3 matrix to store rotation, no translation in the matrix itself
    public Matrix4x4 Rotation = Rotation;
    public Vector3 Translation = Translation;

    public CoordinateFrame() : this(Matrix4x4.Identity, Vector3.Zero)
    {
    }

    public static CoordinateFrame operator *(in CoordinateFrame cf, in CoordinateFrame other)
    {
        return new CoordinateFrame(other.Rotation * cf.Rotation,
            Vector3.TransformNormal(other.Translation, Matrix4x4.Transpose(cf.Rotation)) + cf.Translation); // TODO: Why do we transpose parent rotation here
    }

    public CoordinateFrame Inverse()
    {
        var inverseRotation = Matrix4x4.Transpose(Rotation);

        return new CoordinateFrame(inverseRotation, -Vector3.TransformNormal(Translation, inverseRotation));
    }

    public Vector3 PointToWorldSpace(in Vector3 point)
    {
        return Vector3.TransformNormal(point, Rotation) + Translation;
    }

    public Vector3 PointToObjectSpace(in Vector3 point)
    {
        return Vector3.TransformNormal(point - Translation, Matrix4x4.Transpose(Rotation));
    }

    public Vector3 VectorToWorldSpace(in Vector3 Vector)
    {
        return Vector3.TransformNormal(Vector, Rotation);
    }

    public Vector3 VectorToObjectSpace(in Vector3 Vector)
    {
        return Vector3.TransformNormal(Vector, Matrix4x4.Transpose(Rotation));
    }

    public CoordinateFrame ToObjectSpace(in CoordinateFrame other)
    {
        return Inverse() * other;
    }

    public Vector3 Column(int i)
    {
        return Rotation[i].AsVector3();
    }
}
