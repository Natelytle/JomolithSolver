using System.Numerics;

namespace JomolithSolver.Solver;

public struct SolverBody
{
    public struct SolverBodyDynamicProperties
    {
        public Vector3 IntegratedLinearVelocity;
        public Vector3 IntegratedAngularVelocity;
        public Matrix4x4 Orientation;
        public Vector3 Position;
        public Vector3 LinearVelocity;
        public Vector3 AngularVelocity;
    }

    public struct SolverBodyStaticProperties
    {
        public long BodyUID;
        public int Guid;
        public bool IsStatic;
    }

    public struct SymmetricMatrix
    {
        public Vector3 Diagonals;
        public Vector3 OffDiagonals;

        public static Vector3 operator *(in SymmetricMatrix mat, in Vector3 v)
        {
            return new Vector3(
                mat.Diagonals.X * v.X + mat.OffDiagonals.X * v.Y + mat.OffDiagonals.Y * v.Z,
                mat.OffDiagonals.X * v.X + mat.Diagonals.Y * v.Y + mat.OffDiagonals.Z * v.Z,
                mat.OffDiagonals.Y * v.X + mat.OffDiagonals.Z * v.Y + mat.Diagonals.Z * v.Z
            );
        }
    }

    public struct SolverBodyMassAndInertia
    {
        public Vector3 InertiaDiagonal;
        public float MassInvVelStage;
        public Vector3 InertiaOffDiagonal;
        public float PosToVelMassRatio;

        public float GetInvMassVelStage()
        {
            return MassInvVelStage;
        }

        public float GetInvMassPosStage()
        {
            return MassInvVelStage * PosToVelMassRatio;
        }

        public SymmetricMatrix GetInvInertiaVelStage()
        {
            return new SymmetricMatrix { Diagonals = InertiaDiagonal, OffDiagonals = InertiaOffDiagonal };
        }

        public SymmetricMatrix GetInvInertiaPosStage(float scale)
        {
            var s = scale * PosToVelMassRatio;
            return new SymmetricMatrix { Diagonals = s * InertiaDiagonal, OffDiagonals = s * InertiaOffDiagonal };
        }
    }
}
