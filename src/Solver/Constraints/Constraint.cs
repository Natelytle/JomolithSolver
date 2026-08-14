using System;

namespace JomolithSolver.Solver.Constraints;

public abstract class Constraint
{
    public enum ConstraintType
    {
        Collision,
        Align2Axes,
        BallInSocket,
        AngularVelocity,
        LinearVelocity,
        AchievePosition,
        BodyAngularVelocity,
        LinearSpring,
        LegacyBreakableBallInSocket,
        LegacyAngularVelocity,
        Count
    }

    public enum Convergence
    {
        Converges,
        Diverges,
        Undetermined
    }

    private readonly Body bodyA;
    private readonly Body bodyB;
    private bool broken;

    private readonly ConstraintVariables.ConstraintCache[] cacheData;

    private readonly byte dimensions;
    private readonly ConstraintType type;
    private long uid;

    public Constraint(ConstraintType type, Body a, Body b, byte dimensions)
    {
        this.dimensions = dimensions;
        this.type = type;
        bodyA = a;
        bodyB = b;
        cacheData = new ConstraintVariables.ConstraintCache[dimensions];

        // for (int i = 0; i < cacheData.Length; i++)
        // {
        //     cacheData[i] = new ConstraintVariables.ConstraintCache();
        // }
    }

    public void RestoreCacheAndBuildEquation(Span<ConstraintJacobianPair> jacobian, Span<ConstraintVariables> velStage,
        Span<ConstraintVariables> posStage, Span<float> sorVel, Span<float> sorPos, Span<bool> useBlock,
        in SolverBody.SolverBodyDynamicProperties bodyA, in SolverBody.SolverBodyDynamicProperties bodyB,
        in SolverConfig config, float dt)
    {
        for (var i = 0; i < dimensions; i++)
        {
            velStage[i].MinImpulseValue = float.NegativeInfinity;
            velStage[i].MaxImpulseValue = float.PositiveInfinity;
            posStage[i].MinImpulseValue = float.NegativeInfinity;
            posStage[i].MaxImpulseValue = float.PositiveInfinity;

            jacobian[i].Reset();

            useBlock[i] = config.BlockPGSEnabled;

            GetCache(i).ReadCache(ref velStage[i], ref posStage[i], ref sorVel[i], ref sorPos[i]);

            velStage[i].Impulse *= config.VelCacheDamping;
            posStage[i].Impulse *= config.PosCacheDamping;
        }

        BuildEquation(jacobian, useBlock, velStage, posStage, in bodyA, in bodyB, in config, dt);
    }

    public void Cache(Span<ConstraintVariables> velStage, Span<ConstraintVariables> posStage, Span<float> sorVel, Span<float> sorPos, in SolverConfig config)
    {
        for (var i = 0; i < dimensions; i++) cacheData[i].Cache(ref velStage[i], ref posStage[i], sorVel[i], sorPos[i], type == ConstraintType.Collision, config);
    }

    public void UpdateBrokenState(Span<ConstraintVariables> velStage, Span<ConstraintVariables> posStage,
        in SolverConfig config)
    {
        if (!broken)
            broken = ComputeBrokenState(velStage, posStage, config);
    }

    public abstract Convergence TestPGSConvergence();

    public byte GetDimensions()
    {
        return dimensions;
    }

    public Body GetBodyA()
    {
        return bodyA;
    }

    public Body GetBodyB()
    {
        return bodyB;
    }

    public void SetUID(long id)
    {
        uid = id;
    }

    public bool HasValidUID()
    {
        return uid != 0;
    }

    public long GetUID()
    {
        return uid;
    }

    protected ref ConstraintVariables.ConstraintCache GetCache(int d)
    {
        return ref cacheData[d];
    }

    protected abstract void BuildEquation(Span<ConstraintJacobianPair> jacobian, Span<bool> useBlock,
        Span<ConstraintVariables> velStage, Span<ConstraintVariables> posStage,
        in SolverBody.SolverBodyDynamicProperties bodyA, in SolverBody.SolverBodyDynamicProperties bodyB,
        in SolverConfig config, float dt);

    protected virtual bool ComputeBrokenState(Span<ConstraintVariables> velStage, Span<ConstraintVariables> posStage,
        in SolverConfig config)
    {
        return false;
    }
}
