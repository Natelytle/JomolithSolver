using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace JomolithSolver.Solver.Utils;

public static class SolverUtils
{
    /// <summary>
    ///     Linear interpolation function
    /// </summary>
    /// <param name="x">Value between 0 and 1 to linearly interpolate</param>
    /// <param name="low">Value when x is 0</param>
    /// <param name="high">Value when x is 1</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Lerp(float x, float low, float high)
    {
        return low * (1 - x) + high * x;
    }

    /// <summary>
    ///     Reverse linear interpolation function
    /// </summary>
    /// <param name="x">Value to calculate the function for</param>
    /// <param name="start">Value at which function returns 0</param>
    /// <param name="end">Value at which function returns 1</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float ReverseLerp(float x, float start, float end)
    {
        return Math.Clamp((x - start) / (end - start), 0.0f, 1.0f);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static VirtualDisplacement ApplyImpulse(in VirtualDisplacement virD, float impulse, in EffectiveMass eff)
    {
        return new VirtualDisplacement(virD.Linear + impulse * eff.Linear, virD.Angular + impulse * eff.Angular);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float ProjectOntoJacobian(in ConstraintJacobianPair j, in VirtualDisplacement va,
        in VirtualDisplacement vb)
    {
        return Vector3.Dot(j.A.Linear, va.Linear) + Vector3.Dot(j.A.Angular, va.Angular) +
               Vector3.Dot(j.B.Linear, vb.Linear) + Vector3.Dot(j.B.Angular, vb.Angular);
    }

    public static void GenerateOrthonormalBasis(out Vector3 t1, out Vector3 t2, in Vector3 n)
    {
        if (Math.Abs(n.X) < 0.9f)
            t1 = Vector3.Normalize(Vector3.Cross(n, new Vector3(1, 0, 0)));
        else
            t1 = Vector3.Normalize(Vector3.Cross(n, new Vector3(0, 1, 0)));
        t2 = Vector3.Cross(t1, n);
    }
}
