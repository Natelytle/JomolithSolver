
using System.Numerics;
using Godot;
using Vector3 = Godot.Vector3;

namespace JomolithSolver.Utils;

public static class Vector3Extensions
{
    public static System.Numerics.Vector3 ToNumerics(this Vector3 vector) => new(vector.X, vector.Y, vector.Z);
    public static Vector3 ToGodot(this System.Numerics.Vector3 vector) => new(vector.X, vector.Y, vector.Z);

    public static Matrix4x4 ToMatrix(this Basis b) => new(
        b[0, 0], b[1, 0], b[2, 0], 0,
        b[0, 1], b[1, 1], b[2, 1], 0,
        b[0, 2], b[1, 2], b[2, 2], 0,
        0,       0,       0,       1
    );

    public static Basis ToBasis(this Matrix4x4 m) => new(
        m.M11, m.M12, m.M13,
        m.M21, m.M22, m.M23,
        m.M31, m.M32, m.M33
    );
}
