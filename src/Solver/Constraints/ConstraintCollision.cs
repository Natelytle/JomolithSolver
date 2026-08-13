using System;
using System.Numerics;
using JomolithSolver.Solver.Utils;

namespace JomolithSolver.Solver.Constraints;

public class ConstraintCollision : Constraint
{
    public ConstraintCollision(Body a, Body b) : base(ConstraintType.Collision, a, b, 3)
    {
        Depth = 0;
        Friction = 0;
        Restitution = 0;
        CachedTangent1 = new Vector3(1, 0, 0);
    }

    public Vector3 Normal { get; set; }
    public Vector3 PointA { get; set; }
    public float Depth { get; set; }
    public float Friction { get; set; }
    public float Restitution { get; set; }
    public Vector3 CachedTangent1 { get; set; }

    protected override void BuildEquation(Span<ConstraintJacobianPair> jacobian, Span<bool> useBlock, Span<ConstraintVariables> velStage, Span<ConstraintVariables> posStage,
        in SolverBody.SolverBodyDynamicProperties bodyA,
        in SolverBody.SolverBodyDynamicProperties bodyB, in SolverConfig config, float dt)
    {
        useBlock[0] = false;
        useBlock[1] = false;
        useBlock[2] = false;

        var relA = PointA - bodyA.Position;
        var relB = PointA + Depth * Normal - bodyB.Position;

        jacobian[0].A.Linear = -Normal;
        jacobian[0].B.Linear = Normal;
        jacobian[0].A.Angular = -Vector3.Cross(relA, Normal);
        jacobian[0].B.Angular = Vector3.Cross(relB, Normal);

        var intDeltaV = bodyA.IntegratedLinearVelocity + Vector3.Cross(bodyA.IntegratedAngularVelocity, relA) -
                        (bodyB.IntegratedLinearVelocity + Vector3.Cross(bodyB.IntegratedAngularVelocity, relB));
        var intNormalVel = Vector3.Dot(intDeltaV, Normal);

        var prevDeltaV = bodyA.LinearVelocity + Vector3.Cross(bodyA.AngularVelocity, relA) -
                         (bodyB.LinearVelocity + Vector3.Cross(bodyB.AngularVelocity, relB));
        var prevNormalVel = Vector3.Dot(prevDeltaV, Normal);

        velStage[0].Reaction = intNormalVel + Restitution * (prevNormalVel < config.CollisionRestitutionThreshold ? 0.0f : prevNormalVel);
        velStage[0].MinImpulseValue = 0.0f;
        velStage[0].MaxImpulseValue = float.PositiveInfinity;

        var intTangVel = intDeltaV - intNormalVel * Normal;
        var prevTangVel = prevDeltaV - prevNormalVel * Normal;
        var prevTangSpeedSq = Vector3.Dot(prevTangVel, prevTangVel);

        Vector3 t1, t2;
        float frictionBound;

        if (prevTangSpeedSq < config.CollisionFrictionStaticToDynamicThreshold *
            config.CollisionFrictionStaticToDynamicThreshold)
        {
            SolverUtils.GenerateOrthonormalBasis(out t1, out t2, Normal);
            frictionBound = config.CollisionFrictionStaticScale * Friction;

            var cachedT2 = Vector3.Cross(CachedTangent1, Normal);
            var velImpulse = velStage[1].Impulse * CachedTangent1 + velStage[2].Impulse * cachedT2;
            var posImpulse = posStage[1].Impulse * CachedTangent1 + posStage[2].Impulse * cachedT2;

            velStage[1].Reaction = Vector3.Dot(intTangVel, t1);
            velStage[1].Impulse = Vector3.Dot(velImpulse, t1);
            velStage[1].MinImpulseValue = -frictionBound;
            velStage[1].MaxImpulseValue = frictionBound;

            velStage[2].Reaction = Vector3.Dot(intTangVel, t2);
            velStage[2].Impulse = Vector3.Dot(velImpulse, t2);
            velStage[2].MinImpulseValue = -frictionBound;
            velStage[2].MaxImpulseValue = frictionBound;

            posStage[1].Reaction = 0;
            posStage[1].Impulse = Vector3.Dot(posImpulse, t1);
            posStage[1].MinImpulseValue = -frictionBound;
            posStage[1].MaxImpulseValue = frictionBound;

            posStage[2].Reaction = 0;
            posStage[2].Impulse = Vector3.Dot(posImpulse, t2);
            posStage[2].MinImpulseValue = -frictionBound;
            posStage[2].MaxImpulseValue = frictionBound;
        }
        else
        {
            t1 = Vector3.Normalize(prevTangVel);
            t2 = Vector3.Cross(t1, Normal);
            frictionBound = config.CollisionFrictionDynamicScale * Friction;

            var cachedT2 = Vector3.Cross(CachedTangent1, Normal);
            var velImpulse = velStage[1].Impulse * CachedTangent1 + velStage[2].Impulse * cachedT2;

            velStage[1].Reaction = Vector3.Dot(intTangVel, t1);
            velStage[1].Impulse = Vector3.Dot(velImpulse, t1);
            velStage[1].MinImpulseValue = 0.0f;
            velStage[1].MaxImpulseValue = frictionBound;

            velStage[2].Reaction = 0.0f;
            velStage[2].Impulse = 0.0f;
            velStage[2].MinImpulseValue = 0.0f;
            velStage[2].MaxImpulseValue = 0.0f;

            posStage[1] = new ConstraintVariables();
            posStage[2] = new ConstraintVariables();
            posStage[1].MinImpulseValue = 0.0f;
            posStage[2].MinImpulseValue = 0.0f;
            posStage[2].MinImpulseValue = 0.0f;
            posStage[2].MaxImpulseValue = 0.0f;
        }

        CachedTangent1 = t1;

        jacobian[1].A.Linear = -t1;
        jacobian[1].B.Linear = t1;
        jacobian[1].A.Angular = -Vector3.Cross(relA, t1);
        jacobian[1].B.Angular = Vector3.Cross(relB, t1);

        jacobian[2].A.Linear = -t2;
        jacobian[2].B.Linear = t2;
        jacobian[2].A.Angular = -Vector3.Cross(relA, t2);
        jacobian[2].B.Angular = Vector3.Cross(relB, t2);

        posStage[0].Reaction = -config.CollisionPenetrationResolutionDamping * (Depth + config.CollisionPenetrationMargin);
        posStage[0].MinImpulseValue = 0.0f;
        posStage[0].MaxImpulseValue = float.PositiveInfinity;
    }

    public override Convergence TestPGSConvergence()
    {
        throw new NotImplementedException();
    }
}
