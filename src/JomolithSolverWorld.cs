using System;
using System.Collections.Generic;
using System.Numerics;
using Godot;
using JomolithSolver.Solver;
using JomolithSolver.Solver.Constraints;
using JomolithSolver.Utils;
using static JomolithSolver.Solver.Solver;
using Vector3 = Godot.Vector3;

namespace JomolithSolver;

[GlobalClass]
public partial class JomolithSolverWorld : Node
{
    private long nextUid;

    private readonly CollisionWorld collisionWorld = new();
    private readonly ContactManager contactManager = new();
    private readonly Solver.Solver solver = new(new SolverConfig());

    private SimBodyInput[] solverInputs = null!;
    private SimBodyOutput[] solverOutputs = null!;
    private readonly List<Constraint> constraintsBuffer = [];
    private readonly List<BodyPairIndices> pairsBuffer = [];
    private readonly List<byte> dimsBuffer = [];

    private readonly Dictionary<long, int> uidToIndex = new();

    private readonly HashSet<JomolithRigidBody> registeredBodies = [];
    private readonly HashSet<Body> requiredBodies = []; // All the bodies the solver needs to care about (excludes static bodies not touching anything)

    public override void _Ready()
    {
        solverInputs = Array.Empty<SimBodyInput>();
        solverOutputs = Array.Empty<SimBodyOutput>();
        AddToGroup("SolverWorld");
    }

    public override void _PhysicsProcess(double delta)
    {
        // Run callback where you can set positions before collisions are detected.
        foreach (var rb in registeredBodies)
        {
            rb._PrePhysicsProcess(delta);

            // Apply our godot node's new position to the solver body.
            // This should allow changes to position to occur at runtime.
            rb.SolverBody.SetWorldCFrame(rb.Basis.ToMatrix(), rb.GlobalPosition.ToNumerics());
        }

        UpdateCollisionsAndRequiredBodies();
        CreateSolverInputs();
        StepPhysics((float)delta);
        ApplySolverOutputs();
    }

    private void UpdateCollisionsAndRequiredBodies()
    {
        List<Body> bodies = new(registeredBodies.Count);

        foreach (var rb in registeredBodies)
        {
            bodies.Add(rb.SolverBody);
        }

        collisionWorld.SyncTransforms(bodies);
        collisionWorld.DetectCollisions();

        var freshContacts = new Dictionary<ContactManager.PairKey, List<ContactPoint>>();
        var bodyLookup = new Dictionary<ContactManager.PairKey, (Body A, Body B)>();
        collisionWorld.ExtractContacts(freshContacts, bodyLookup);
        contactManager.Update(freshContacts, bodyLookup);

        // All unanchored bodies are required
        foreach (var rb in registeredBodies)
        {
            if (!rb.Anchored) requiredBodies.Add(rb.SolverBody);
        }

        // All colliding bodies are required
        foreach (var kv in bodyLookup)
        {
            requiredBodies.Add(kv.Value.A);
            requiredBodies.Add(kv.Value.B);
        }
    }

    private void CreateSolverInputs()
    {
        int bodyCount = requiredBodies.Count;

        if (solverInputs.Length != bodyCount)
        {
            solverInputs = new SimBodyInput[bodyCount];
            solverOutputs = new SimBodyOutput[bodyCount];
        }

        int index = 0;
        uidToIndex.Clear();

        foreach (var body in requiredBodies)
        {
            // Apply gravity
            body.AccumulateForce(new System.Numerics.Vector3(0, -(float)ProjectSettings.GetSetting("physics/3d/default_gravity") * body.Mass, 0));

            uidToIndex[body.Uid] = index;

            solverInputs[index].Position = body.GetWorldCFrame().Translation;
            solverInputs[index].Orientation = body.GetWorldCFrame().Rotation;

            solverInputs[index].LinearVelocity = body.LinearVelocity;
            solverInputs[index].AngularVelocity = body.AngularVelocity;

            solverInputs[index].ExternalForce = body.ExternalForce;
            solverInputs[index].ExternalTorque = body.ExternalTorque;
            solverInputs[index].ExternalImpulse = body.ExternalImpulse;
            solverInputs[index].ExternalAngularImpulse = body.ExternalRotationalImpulse;

            solverInputs[index].MassInv = body.IsStatic ? 0.0f : 1.0f / body.GetBranchMass();
            solverInputs[index].InertiaInv = new Matrix4x4();
            if (Matrix4x4.Invert(body.GetBranchInertiaWorldAtPoint(solverInputs[index].Position), out var inverseInertia))
            {
                solverInputs[index].InertiaInv = inverseInertia;
            }
            solverInputs[index].EffectiveMassMultiplier = body.IsStatic ? 0.0f : 1.0f;

            index++;
        }
    }

    private void StepPhysics(float delta)
    {
        constraintsBuffer.Clear();
        pairsBuffer.Clear();
        dimsBuffer.Clear();

        int collisionCount = contactManager.GatherConstraints(constraintsBuffer, pairsBuffer, dimsBuffer, uidToIndex);

        solver.Solve(
            solverInputs,
            constraintsBuffer.ToArray(),
            pairsBuffer.ToArray(),
            dimsBuffer.ToArray(),
            collisionCount,
            delta,
            out solverOutputs
        );
    }

    private void ApplySolverOutputs()
    {
        foreach (var rb in registeredBodies)
        {
            // Only apply outputs for bodies that simulated this frame.
            if (rb.Anchored || !uidToIndex.TryGetValue(rb.SolverBody.Uid, out int index))
                continue;

            var outData = solverOutputs[index];

            rb.Position = new Vector3(outData.Position.X, outData.Position.Y, outData.Position.Z);
            rb.Basis = outData.Orientation.ToBasis();
            rb.Basis = rb.Basis.ToMatrix().ToBasis().ToMatrix().ToBasis();

            rb.LinearVelocity = new Vector3(outData.LinearVelocity.X, outData.LinearVelocity.Y, outData.LinearVelocity.Z);
            rb.AngularVelocity = new Vector3(outData.AngularVelocity.X, outData.AngularVelocity.Y, outData.AngularVelocity.Z);
        }

        foreach (var b in requiredBodies)
        {
            b.ClearAccumulators();
        }
    }

    public void RegisterBody(JomolithRigidBody rb)
    {
        registeredBodies.Add(rb);
        collisionWorld.AddBody(rb.SolverBody);
        rb.SolverBody.Uid = nextUid++;
    }

    public void UnregisterBody(JomolithRigidBody rb)
    {
        registeredBodies.Remove(rb);
        collisionWorld.RemoveBody(rb.SolverBody);
    }
}
