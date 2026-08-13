using System;
using System.Numerics;
using BulletSharp.SoftBody;
using JomolithSolver.Solver.Constraints;

namespace JomolithSolver.Solver;

public class Solver(SolverConfig config)
{
    private SolverConfig config = config;

    public void Solve(
        SimBodyInput[] bodies,
        Constraint[] constraints,
        BodyPairIndices[] pairs,
        byte[] dimensions,
        int collisionCount,
        float dt,
        out SimBodyOutput[] outputs
    )
    {
        int bodyCount = bodies.Length;
        int constraintCount = constraints.Length;

        int totalDim = 0;
        foreach (var d in dimensions)
        {
            totalDim += d;
        }

        // Step 1: Build mass and inertia arrays
        SolverBody.SolverBodyMassAndInertia[] massAndInertia = new SolverBody.SolverBodyMassAndInertia[bodyCount];

        for (int i = 0; i < bodyCount; i++)
        {
            SimBodyInput b = bodies[i];
            Matrix4x4 ii = b.InertiaInv;
            massAndInertia[i].MassInvVelStage = b.MassInv;
            massAndInertia[i].PosToVelMassRatio = b.MassInv > 0.0f ? float.Pow(b.MassInv, config.StabilizationMassReductionPower) / b.MassInv : 0.0f;
            massAndInertia[i].InertiaDiagonal = new Vector3(ii[0, 0], ii[1, 1], ii[2, 2]);
            massAndInertia[i].InertiaOffDiagonal = new Vector3(ii[0, 1], ii[0, 2], ii[1, 2]);
        }

        // Step 2: Integrate velocities and initialize body properties
        SolverBody.SolverBodyDynamicProperties[] bodyProps = new SolverBody.SolverBodyDynamicProperties[bodyCount];

        for (int i = 0; i < bodyCount; i++)
        {
            SimBodyInput b = bodies[i];
            bodyProps[i].Position = b.Position;
            bodyProps[i].Orientation = b.Orientation;
            bodyProps[i].LinearVelocity = b.LinearVelocity;
            bodyProps[i].AngularVelocity = b.AngularVelocity;
            IntegrateVelocities(ref bodyProps[i], massAndInertia[i], b, dt, config);
        }

        // Step 3: Allocate constraint arrays
        ConstraintJacobianPair[] jacobians = new ConstraintJacobianPair[totalDim];
        ConstraintVariables[] velStage = new ConstraintVariables[totalDim];
        ConstraintVariables[] posStage = new ConstraintVariables[totalDim];
        float[] sorVel = new float[totalDim];
        float[] sorPos = new float[totalDim];
        bool[] useBlock = new bool[totalDim];

        for (int i = 0; i < totalDim; i++)
        {
            sorVel[i] = 1;
            sorPos[i] = 1;
            useBlock[i] = config.BlockPGSEnabled;
        }

        // Step 4: Build constraint equations
        int offset = 0;
        for (int c = 0; c < constraintCount; c++)
        {
            byte d = dimensions[c];
            int iA = pairs[c].First;
            int iB = pairs[c].Second;

            SolverBody.SolverBodyDynamicProperties emptyProps = new SolverBody.SolverBodyDynamicProperties();

            SolverBody.SolverBodyDynamicProperties bA = (iA >= 0 && iA < bodyCount) ? bodyProps[iA] : emptyProps;
            SolverBody.SolverBodyDynamicProperties bB = (iB >= 0 && iB < bodyCount) ? bodyProps[iB] : emptyProps;

            constraints[c].RestoreCacheAndBuildEquation(
                jacobians.AsSpan(offset),
                velStage.AsSpan(offset),
                posStage.AsSpan(offset),
                sorVel.AsSpan(offset),
                sorPos.AsSpan(offset),
                useBlock.AsSpan(offset),
                bA, bB, config, dt
            );

            offset += d;
        }

        // Step 5: Compute effective masses
        EffectiveMassPair[] effVel = new EffectiveMassPair[totalDim];
        EffectiveMassPair[] effPos = new EffectiveMassPair[totalDim];
        SolverKernel.ComputeEffectiveMasses(effVel, effPos, jacobians, pairs, dimensions, massAndInertia, config);

        // Step 6: Precondition
        ConstraintJacobianPair[] preJacVel = new ConstraintJacobianPair[totalDim];
        ConstraintJacobianPair[] preJacPos = new ConstraintJacobianPair[totalDim];
        SolverKernel.PreconditionConstraintEquations(preJacVel, preJacPos, velStage, posStage, jacobians, pairs, dimensions, useBlock, sorVel, sorPos, effVel, effPos, config);

        // Step 7: Apply effective mass multipliers (sleeping bodies)
        float[] multipliers = new float[bodyCount];
        for (int i = 0; i < bodyCount; i++)
        {
            multipliers[i] = bodies[i].EffectiveMassMultiplier;
        }

        SolverKernel.ApplyEffectiveMassMultipliers(effVel, effPos, pairs, dimensions, multipliers, config);

        // Step 8: Init virtual displacements
        VirtualDisplacementArray virDVel = new VirtualDisplacementArray(bodyCount);
        VirtualDisplacementArray virDPos = new VirtualDisplacementArray(bodyCount);
        virDVel.Reset();
        virDPos.Reset();
        SolverKernel.InitVirtualDisplacements(ref virDVel, ref virDPos, velStage, posStage, effVel, effPos, pairs, dimensions, config);

        // Step 9: Run PGS Kernel
        SolverKernel.SolveKernel(velStage, posStage, ref virDVel, ref virDPos, preJacVel, preJacPos, effVel, effPos, pairs, dimensions, collisionCount, config);

        // Step 10: Cache constraint results
        offset = 0;
        for (int c = 0; c < constraintCount; c++)
        {
            byte d = dimensions[c];
            constraints[c].UpdateBrokenState(velStage.AsSpan(offset), posStage.AsSpan(offset), config);
            constraints[c].Cache(velStage.AsSpan(offset), posStage.AsSpan(offset), sorVel.AsSpan(offset), sorPos.AsSpan(offset), config);
            offset += d;
        }

        // Step 11: Integrate positions and write outputs
        outputs = new SimBodyOutput[bodyCount];
        for (int i = 0; i < bodyCount; i++)
        {
            IntegratePositions(ref bodyProps[i], virDVel[i], virDPos[i], dt);
            outputs[i].Position = bodyProps[i].Position;
            outputs[i].Orientation = bodyProps[i].Orientation;
            outputs[i].LinearVelocity = bodyProps[i].LinearVelocity;
            outputs[i].AngularVelocity = bodyProps[i].AngularVelocity;
        }
    }

    private static void IntegrateVelocities(
        ref SolverBody.SolverBodyDynamicProperties props,
        in SolverBody.SolverBodyMassAndInertia mass,
        in SimBodyInput input,
        float dt,
        in SolverConfig config
    )
    {
        props.IntegratedLinearVelocity = input.LinearVelocity + mass.MassInvVelStage * (input.ExternalForce * dt + input.ExternalImpulse);

        var dampFactor = float.Exp(-config.AngularDamping * dt);
        props.IntegratedAngularVelocity = dampFactor * input.AngularVelocity + Vector3.TransformNormal(input.ExternalTorque * dt + input.ExternalAngularImpulse, input.InertiaInv);
    }

    private static void IntegratePositions(
        ref SolverBody.SolverBodyDynamicProperties props,
        in VirtualDisplacement velDelta,
        in VirtualDisplacement posDelta,
        float dt
    )
    {
        props.LinearVelocity = props.IntegratedLinearVelocity + velDelta.Linear;
        props.AngularVelocity = props.IntegratedAngularVelocity + velDelta.Angular;

        props.Position += posDelta.Linear;

        var angle = posDelta.Angular.Length();

        if (posDelta.Angular.Length() > 1e-10f)
        {
            var axis = posDelta.Angular / angle;
            var rot = Matrix4x4.CreateFromQuaternion(Quaternion.CreateFromAxisAngle(axis, angle));
            props.Orientation = rot * props.Orientation;
        }

        props.Position += props.LinearVelocity * dt;

        angle = props.AngularVelocity.Length();

        if (angle > 1e-10f)
        {
            var dtAngle = angle * dt;
            var axis = props.AngularVelocity / angle;
            var rot = Matrix4x4.CreateFromQuaternion(Quaternion.CreateFromAxisAngle(axis, dtAngle));
            props.Orientation = rot * props.Orientation;
        }

        var c0 = Vector3.Normalize(props.Orientation[0].AsVector3());
        var c2 = Vector3.Normalize(Vector3.Cross(c0, props.Orientation[1].AsVector3()));
        var c1 = Vector3.Cross(c2, c0);

        props.Orientation = new Matrix4x4(
            c0.X, c0.Y, c0.Z, 0,
            c1.X, c1.Y, c1.Z, 0,
            c2.X, c2.Y, c2.Z, 0,
            0, 0, 0, 1
        );
    }

    public struct SimBodyInput()
    {
        public Vector3 Position;
        public Matrix4x4 Orientation;
        public Vector3 LinearVelocity;
        public Vector3 AngularVelocity;

        public Vector3 ExternalForce;
        public Vector3 ExternalTorque;
        public Vector3 ExternalImpulse;
        public Vector3 ExternalAngularImpulse;

        public Matrix4x4 InertiaInv;

        public float MassInv = 0.0f; // default: static (anchored)

        // 1.0 = fully simulated, 0.0 = static/sleeping (collapses mass to inf)
        public float EffectiveMassMultiplier = 1.0f;
    }

    public struct SimBodyOutput
    {
        public Vector3 Position;
        public Matrix4x4 Orientation;
        public Vector3 LinearVelocity;
        public Vector3 AngularVelocity;
    }
}
