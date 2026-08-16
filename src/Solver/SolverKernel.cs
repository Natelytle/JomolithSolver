
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace JomolithSolver.Solver;

public static class SolverKernel
{
    public static void SolveKernel(
        ConstraintVariables[] velStage,
        ConstraintVariables[] posStage,
        VirtualDisplacement[] virDVel,
        VirtualDisplacement[] virDPos,
        ConstraintJacobianPair[] preJacVel,
        ConstraintJacobianPair[] preJacPos,
        EffectiveMassPair[] effVel,
        EffectiveMassPair[] effPos,
        BodyPairIndices[] pairs,
        byte[] dimensions,
        int collisionCount,
        in SolverConfig config
    )
    {
        int constraintCount = dimensions.Length;
        int pureCount = constraintCount - collisionCount;

        for (int k = 0; k < config.PgsIterations; k++)
        {
            int offset = 0;

            for (int c = 0; c < pureCount; c++)
            {
                byte d = dimensions[c];
                int iA = pairs[c].First;
                int iB = pairs[c].Second;

                // Yatta!!!
                ref VirtualDisplacement vaVel = ref virDVel[iA];
                ref VirtualDisplacement vbVel = ref virDVel[iB];
                ref VirtualDisplacement vaPos = ref virDPos[iA];
                ref VirtualDisplacement vbPos = ref virDPos[iB];

                switch (d)
                {
                    case 1:
                        UpdateConstraint1D(
                            ref velStage[offset], ref posStage[offset],
                            ref vaVel, ref vbVel, ref vaPos, ref vbPos,
                            preJacVel[offset], preJacPos[offset],
                            effVel[offset], effPos[offset]
                        );
                        break;
                    case 2:
                        UpdateConstraint2D(
                            velStage.AsSpan(offset, 2), posStage.AsSpan(offset, 2),
                            ref vaVel, ref vbVel, ref vaPos, ref vbPos,
                            preJacVel.AsSpan(offset, 2), preJacPos.AsSpan(offset, 2),
                            effVel.AsSpan(offset, 2), effPos.AsSpan(offset, 2)
                        );
                        break;
                    case 3:
                        UpdateConstraint3D(
                            velStage.AsSpan(offset, 3), posStage.AsSpan(offset, 3),
                            ref vaVel, ref vbVel, ref vaPos, ref vbPos,
                            preJacVel.AsSpan(offset, 3), preJacPos.AsSpan(offset, 3),
                            effVel.AsSpan(offset, 3), effPos.AsSpan(offset, 3),
                            false
                        );
                        break;
                }

                offset += d;
            }

            // Collision constraints, always dim 3 with friction cone
            for (int c = pureCount; c < constraintCount; c++)
            {
                int iA = pairs[c].First;
                int iB = pairs[c].Second;

                UpdateConstraint3D(
                    velStage.AsSpan(offset, 3), posStage.AsSpan(offset, 3),
                    ref virDVel[iA], ref virDVel[iB], ref virDPos[iA], ref virDPos[iB],
                    preJacVel.AsSpan(offset, 3), preJacPos.AsSpan(offset, 3),
                    effVel.AsSpan(offset, 3), effPos.AsSpan(offset, 3),
                    true
                );

                offset += 3;
            }

            // if (velStage.Length >= Math.Max(offset, 3))
            //     Console.WriteLine($"[solve] normalImpulse={velStage[offset - 3].Impulse:F4} t1={velStage[offset - 2].Impulse:F4} t2={velStage[offset - 1].Impulse:F4}");
        }
    }

    public static void ComputeEffectiveMasses(
        EffectiveMassPair[] effVel,
        EffectiveMassPair[] effPos,
        ConstraintJacobianPair[] jacobians,
        BodyPairIndices[] pairs,
        byte[] dimensions,
        SolverBody.SolverBodyMassAndInertia[] massAndInertia,
        in SolverConfig config
    )
    {
        int offset = 0;

        for (int i = 0; i < dimensions.Length; i++)
        {
            byte d = dimensions[i];
            SolverBody.SolverBodyMassAndInertia mA = massAndInertia[pairs[i].First];
            SolverBody.SolverBodyMassAndInertia mB = massAndInertia[pairs[i].Second];

            for (byte j = 0; j < d; j++)
            {
                ConstraintJacobianPair jac = jacobians[offset + j];

                // Velocity stage: W * J^t using full mass
                EffectiveMass emAVel = new EffectiveMass(mA.GetInvMassVelStage() * jac.A.Linear, mA.GetInvInertiaVelStage() * jac.A.Angular);
                EffectiveMass emBVel = new EffectiveMass(mB.GetInvMassVelStage() * jac.B.Linear, mB.GetInvInertiaVelStage() * jac.B.Angular);
                effVel[offset + j] = new EffectiveMassPair(emAVel, emBVel);

                // Position stage: use reduced mass for stabilization
                EffectiveMass emAPos = new EffectiveMass(mA.GetInvMassPosStage() * jac.A.Linear, mA.GetInvInertiaPosStage(config.StabilizationInertiaScale) * jac.A.Angular);
                EffectiveMass emBPos = new EffectiveMass(mB.GetInvMassPosStage() * jac.B.Linear, mB.GetInvInertiaPosStage(config.StabilizationInertiaScale) * jac.B.Angular);
                effPos[offset + j] = new EffectiveMassPair(emAPos, emBPos);
            }

            offset += d;
        }
    }

    public static void ApplyEffectiveMassMultipliers(
        EffectiveMassPair[] effVel,
        EffectiveMassPair[] effPos,
        BodyPairIndices[] pairs,
        byte[] dimensions,
        float[] multipliers,
        in SolverConfig config
    )
    {
        int offset = 0;

        for (int i = 0; i < dimensions.Length; i++)
        {
            byte d = dimensions[i];
            float mA = multipliers[pairs[i].First];
            float mB = multipliers[pairs[i].Second];

            for (byte j = 0; j < d; j++)
            {
                effVel[offset+j].ApplyMultipliers(mA, mB);
                effPos[offset+j].ApplyMultipliers(mA, mB);
            }

            offset += d;
        }
    }

    public static void PreconditionConstraintEquations(
        ConstraintJacobianPair[] preJacVel,
        ConstraintJacobianPair[] preJacPos,
        ConstraintVariables[] velStage,
        ConstraintVariables[] posStage,
        ConstraintJacobianPair[] jacobians,
        BodyPairIndices[] pairs,
        byte[] dimensions,
        bool[] useBlock,
        float[] sorVel,
        float[] sorPos,
        EffectiveMassPair[] effVel,
        EffectiveMassPair[] effPos,
        in SolverConfig config
    )
    {
        int offset = 0;

        for (int i = 0; i < dimensions.Length; i++)
        {
            byte d = dimensions[i];

            switch (d)
            {
                case 1:
                    PreconditionStage1D(ref velStage[offset], ref preJacVel[offset], ref jacobians[offset], effVel[offset], offset);
                    PreconditionStage1D(ref posStage[offset], ref preJacPos[offset], ref jacobians[offset], effPos[offset], offset);
                    break;
                case 2:
                    PreconditionStage2D(velStage.AsSpan(offset), preJacVel.AsSpan(offset), jacobians.AsSpan(offset), effVel.AsSpan(offset), offset);
                    PreconditionStage2D(posStage.AsSpan(offset), preJacPos.AsSpan(offset), jacobians.AsSpan(offset), effPos.AsSpan(offset), offset);
                    break;
                case 3:
                    PreconditionStage3D(velStage.AsSpan(offset), preJacVel.AsSpan(offset), jacobians.AsSpan(offset), effVel.AsSpan(offset), offset);
                    PreconditionStage3D(posStage.AsSpan(offset), preJacPos.AsSpan(offset), jacobians.AsSpan(offset), effPos.AsSpan(offset), offset);
                    break;
            }

            offset += d;
        }

        return;

        void PreconditionStage1D(
            ref ConstraintVariables stage,
            ref ConstraintJacobianPair preJac,
            ref ConstraintJacobianPair jacobian,
            in EffectiveMassPair eff,
            int sorOffset
        )
        {
            float diag = jacobian.Dot(eff);
            float inv = InvertBlock1(diag);
            float sor = sorVel[sorOffset];

            preJac.A.Linear = sor * inv * jacobian.A.Linear;
            preJac.A.Angular = sor * inv * jacobian.A.Angular;
            preJac.B.Linear = sor * inv * jacobian.B.Linear;
            preJac.B.Angular = sor * inv * jacobian.B.Angular;
            stage.Reaction *= sor * inv;
        }

        void PreconditionStage2D(
            Span<ConstraintVariables> stageSpan,
            Span<ConstraintJacobianPair> preJacSpan,
            Span<ConstraintJacobianPair> jacobiansSpan,
            Span<EffectiveMassPair> effSpan,
            int sorOffset
        )
        {
            float d00 = jacobiansSpan[0].Dot(effSpan[0]);
            float d11 = jacobiansSpan[1].Dot(effSpan[1]);
            float d01 = useBlock[offset] && useBlock[offset + 1] ? jacobiansSpan[0].Dot(effSpan[1]) : 0.0f;

            Block2 inv = InvertBlock2(d00, d01, d11);
            float sor0 = sorVel[sorOffset + 0];
            float sor1 = sorVel[sorOffset + 1];

            for (int row = 0; row < 2; row++)
            {
                float sor = row == 0 ? sor0 : sor1;
                float p0 = row == 0 ? inv.M00 : inv.M01;
                float p1 = row == 0 ? inv.M01 : inv.M11;

                preJacSpan[row].A.Linear = sor * (p0 * jacobiansSpan[0].A.Linear + p1 * jacobiansSpan[1].A.Linear);
                preJacSpan[row].A.Angular = sor * (p0 * jacobiansSpan[0].A.Angular + p1 * jacobiansSpan[1].A.Angular);
                preJacSpan[row].B.Linear = sor * (p0 * jacobiansSpan[0].B.Linear + p1 * jacobiansSpan[1].B.Linear);
                preJacSpan[row].B.Angular = sor * (p0 * jacobiansSpan[0].B.Angular + p1 * jacobiansSpan[1].B.Angular);
            }

            float r0 = stageSpan[0].Reaction;
            float r1 = stageSpan[1].Reaction;

            stageSpan[0].Reaction = sor0 * (inv.M00 * r0 + inv.M01 * r1);
            stageSpan[1].Reaction = sor1 * (inv.M01 * r0 + inv.M11 * r1);
        }

        void PreconditionStage3D(
            Span<ConstraintVariables> stageSpan,
            Span<ConstraintJacobianPair> preJacSpan,
            Span<ConstraintJacobianPair> jacobiansSpan,
            Span<EffectiveMassPair> effSpan,
            int sorOffset
        )
        {
            float d00 = jacobiansSpan[0].Dot(effSpan[0]);
            float d11 = jacobiansSpan[1].Dot(effSpan[1]);
            float d22 = jacobiansSpan[2].Dot(effSpan[2]);
            float d01 = useBlock[offset] && useBlock[offset + 1] ? jacobiansSpan[0].Dot(effSpan[1]) : 0.0f;
            float d02 = useBlock[offset] && useBlock[offset + 2] ? jacobiansSpan[0].Dot(effSpan[2]) : 0.0f;
            float d12 = useBlock[offset] && useBlock[offset + 2] ? jacobiansSpan[1].Dot(effSpan[2]) : 0.0f;

            Block3 inv = InvertBlock3(d00, d11, d22, d01, d02, d12);

            for (int row = 0; row < 3; row++)
            {
                float sor = sorVel[sorOffset + row];
                ref ConstraintJacobianPair pj = ref preJacSpan[row];
                pj.Reset();

                for (int col = 0; col < 3; col++)
                {
                    float p = inv.M[row, col];
                    pj.A.Linear += p * jacobiansSpan[col].A.Linear;
                    pj.B.Linear += p * jacobiansSpan[col].B.Linear;
                    pj.A.Angular += p * jacobiansSpan[col].A.Angular;
                    pj.B.Angular += p * jacobiansSpan[col].B.Angular;
                }

                pj.A.Linear *= sor;
                pj.A.Angular *= sor;
                pj.B.Linear *= sor;
                pj.B.Angular *= sor;
            }

            float r0 = stageSpan[0].Reaction;
            float r1 = stageSpan[1].Reaction;
            float r2 = stageSpan[2].Reaction;

            for (int row = 0; row < 3; row++)
            {
                float s = sorVel[sorOffset + row];
                stageSpan[row].Reaction = s * (inv.M[row, 0] * r0 + inv.M[row, 1] * r1 + inv.M[row, 2] * r2);
            }
        }
    }

    public static void InitVirtualDisplacements(
        VirtualDisplacement[] virDVel,
        VirtualDisplacement[] virDPos,
        ConstraintVariables[] velStage,
        ConstraintVariables[] posStage,
        EffectiveMassPair[] effVel,
        EffectiveMassPair[] effPos,
        BodyPairIndices[] pairs,
        byte[] dimensions,
        in SolverConfig config
    )
    {
        int offset = 0;

        for (int i = 0; i < dimensions.Length; i++)
        {
            byte d = dimensions[i];
            int iA = pairs[i].First;
            int iB = pairs[i].Second;

            for (byte j = 0; j < d; j++)
            {
                float vI = velStage[offset + j].Impulse;
                float pI = posStage[offset + j].Impulse;
                AddImpulseToVirD(ref virDVel[iA], vI, effVel[offset + j].A);
                AddImpulseToVirD(ref virDVel[iB], vI, effVel[offset + j].B);
                AddImpulseToVirD(ref virDPos[iA], pI, effPos[offset + j].A);
                AddImpulseToVirD(ref virDPos[iB], pI, effPos[offset + j].B);
            }

            offset += d;
        }
    }

    private static float InvertBlock1(float d) => d > 1e-30f ? 1.0f / d : 0.0f;

    private struct Block2
    {
        public float M00, M01, M11;
    }

    private static Block2 InvertBlock2(float d00, float d01, float d11)
    {
        float det = d00 * d11 - d01 * d01;

        if (Math.Abs(det) < 1e-30f)
            return new Block2();

        float inv = 1.0f / det;

        return new Block2
        {
            M00 = d11 * inv,
            M01 = -d01 * inv,
            M11 = d00 * inv
        };
    }

    private struct Block3()
    {
        public readonly float[,] M = new float[3,3];
    }

    private static Block3 InvertBlock3(float d00, float d11, float d22, float d01, float d02, float d12)
    {
        float det = d00 * (d11 * d22 - d12 * d12) - d01 * (d01 * d22 - d12 * d02) + d02 * (d01 * d12 - d11 * d02);

        Block3 r = new Block3();

        if (Math.Abs(det) < 1e-30f)
        {
            return r;
        }

        float inv = 1.0f / det;
        r.M[0,0] = (d11*d22 - d12*d12) * inv;
        r.M[1,1] = (d00*d22 - d02*d02) * inv;
        r.M[2,2] = (d00*d11 - d01*d01) * inv;
        r.M[0,1] = r.M[1,0] = -(d01*d22 - d02*d12) * inv;
        r.M[0,2] = r.M[2,0] = (d01*d12 - d02*d11) * inv;
        r.M[1,2] = r.M[2,1] = -(d00*d12 - d02*d01) * inv;

        return r;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float ProjectVirD(in ConstraintJacobianPair j, in VirtualDisplacement va, in VirtualDisplacement vb)
    {
        return Vector3.Dot(j.A.Linear, va.Linear) + Vector3.Dot(j.A.Angular, va.Angular) + Vector3.Dot(j.B.Linear, vb.Linear) + Vector3.Dot(j.B.Angular, vb.Angular);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddImpulseToVirD(ref VirtualDisplacement virD, float dImpulse, in EffectiveMass eff)
    {
        virD.Linear += dImpulse * eff.Linear;
        virD.Angular += dImpulse * eff.Angular;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SolveDimension(
        ref ConstraintVariables constraint,
        ref VirtualDisplacement virtualDisplacementA, ref VirtualDisplacement virtualDisplacementB,
        in ConstraintJacobianPair jacobianPair,
        in EffectiveMassPair effectiveMass,
        float impulseLimitMultiplier = 1.0f
    )
    {
        // First, we project the virtual displacement of this dimension for the two bodies.
        // This returns a number that tells us how much the constraint is being violated.
        float projectedVirtualDisplacement = ProjectVirD(jacobianPair, virtualDisplacementA, virtualDisplacementB);

        // The impulse value of the constraint on the previous PGS substep.
        float oldImpulse = constraint.Impulse;

        // We set the min and max impulse to apply to the virtual displacement. We default to the constraint's listed min and max impulse values, but
        // for collisions, friction is dependent on how hard the normal constraint is being violated. Therefore, we adjust it using the multipliers.
        float minImpulse =  constraint.MinImpulseValue * impulseLimitMultiplier;
        float maxImpulse =  constraint.MaxImpulseValue * impulseLimitMultiplier;

        // We get the new impulse that should be applied this substep.
        // By adding the old impulse to the reaction (what the constraint says to do to be in line), we get what the impulse should be this frame.
        // We subtract projected virtual displacement to do something, I dunno.
        float newImpulse = Math.Clamp(oldImpulse + constraint.Reaction - projectedVirtualDisplacement, minImpulse, maxImpulse);

        // The change in impulse, obviously.
        float deltaImpulse = newImpulse - oldImpulse;

        constraint.Impulse = newImpulse;

        // We apply the impulses to our virtual displacements, schmooving them along so that we're closer to the solution for the next PGS iteration.
        AddImpulseToVirD(ref virtualDisplacementA, deltaImpulse, effectiveMass.A);
        AddImpulseToVirD(ref virtualDisplacementB, deltaImpulse, effectiveMass.B);
    }

    private static void UpdateConstraint1D(
        ref ConstraintVariables vel,
        ref ConstraintVariables pos,
        ref VirtualDisplacement virDVelA, ref VirtualDisplacement virDVelB,
        ref VirtualDisplacement virDPosA, ref VirtualDisplacement virDPosB,
        in ConstraintJacobianPair preJacVel,
        in ConstraintJacobianPair preJacPos,
        in EffectiveMassPair effVel,
        in EffectiveMassPair effPos
    )
    {
        // We're only solving one dimension, and we're solving it for both velocity and position.
        SolveDimension(ref vel, ref virDVelA, ref virDVelB, preJacVel, effVel);
        SolveDimension(ref pos, ref virDPosA, ref virDPosB, preJacPos, effPos);
    }

    private static void UpdateConstraint2D(
        Span<ConstraintVariables> vel,
        Span<ConstraintVariables> pos,
        ref VirtualDisplacement virDVelA, ref VirtualDisplacement virDVelB,
        ref VirtualDisplacement virDPosA, ref VirtualDisplacement virDPosB,
        Span<ConstraintJacobianPair> preJacVel,
        Span<ConstraintJacobianPair> preJacPos,
        Span<EffectiveMassPair> effVel,
        Span<EffectiveMassPair> effPos
    )
    {
        // First dimension (both velocity and position)
        SolveDimension(ref vel[0], ref virDVelA, ref virDVelB, preJacVel[0], effVel[0]);
        SolveDimension(ref pos[0], ref virDPosA, ref virDPosB, preJacPos[0], effPos[0]);

        // Second dimension (ditto)
        SolveDimension(ref vel[1], ref virDVelA, ref virDVelB, preJacVel[1], effVel[1]);
        SolveDimension(ref pos[1], ref virDPosA, ref virDPosB, preJacPos[1], effPos[1]);
    }

    private static void UpdateConstraint3D(
        Span<ConstraintVariables> vel,
        Span<ConstraintVariables> pos,
        ref VirtualDisplacement virDVelA, ref VirtualDisplacement virDVelB,
        ref VirtualDisplacement virDPosA, ref VirtualDisplacement virDPosB,
        Span<ConstraintJacobianPair> preJacVel,
        Span<ConstraintJacobianPair> preJacPos,
        Span<EffectiveMassPair> effVel,
        Span<EffectiveMassPair> effPos,
        bool isCollision
    )
    {
        Solve3(vel, ref virDVelA, ref virDVelB, preJacVel, effVel);
        Solve3(pos, ref virDPosA, ref virDPosB, preJacPos, effPos);

        return;

        void Solve3(
            Span<ConstraintVariables> vars,
            ref VirtualDisplacement virDA,
            ref VirtualDisplacement virDB,
            Span<ConstraintJacobianPair> preJac,
            Span<EffectiveMassPair> effMass)
        {
            float p0 = ProjectVirD(preJac[0], virDA, virDB);
            float p1 = ProjectVirD(preJac[1], virDA, virDB);
            float p2 = ProjectVirD(preJac[2], virDA, virDB);

            float old0 = vars[0].Impulse, old1 = vars[1].Impulse, old2 = vars[2].Impulse;
            float min0 = vars[0].MinImpulseValue, max0 = vars[0].MaxImpulseValue;
            float min1 = vars[1].MinImpulseValue, max1 = vars[1].MaxImpulseValue;
            float min2 = vars[2].MinImpulseValue, max2 = vars[2].MaxImpulseValue;

            if (isCollision)
            {
                float normalImpulse = old0;
                min1 *= normalImpulse; max1 *= normalImpulse;
                min2 *= normalImpulse; max2 *= normalImpulse;
            }

            float n0 = Math.Clamp(old0 + vars[0].Reaction - p0, min0, max0);
            float n1 = Math.Clamp(old1 + vars[1].Reaction - p1, min1, max1);
            float n2 = Math.Clamp(old2 + vars[2].Reaction - p2, min2, max2);

            float d0 = n0 - old0, d1 = n1 - old1, d2 = n2 - old2;

            vars[0].Impulse = n0;
            vars[1].Impulse = n1;
            vars[2].Impulse = n2;

            AddImpulse3DToVirD(ref virDA, d0, d1, d2, effMass[0].A, effMass[1].A, effMass[2].A);
            AddImpulse3DToVirD(ref virDB, d0, d1, d2, effMass[0].B, effMass[1].B, effMass[2].B);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddImpulse3DToVirD(ref VirtualDisplacement virD, float d0, float d1, float d2,
        in EffectiveMass eff0, in EffectiveMass eff1, in EffectiveMass eff2)
    {
        virD.Linear += d0 * eff0.Linear + d1 * eff1.Linear + d2 * eff2.Linear;
        virD.Angular += d0 * eff0.Angular + d1 * eff1.Angular + d2 * eff2.Angular;
    }
}
