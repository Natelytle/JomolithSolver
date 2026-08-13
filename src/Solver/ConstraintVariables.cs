using System;
using System.Numerics;
using JomolithSolver.Solver.Utils;

namespace JomolithSolver.Solver;

public struct ConstraintVariables
{
    public float MinImpulseValue = float.NegativeInfinity;
    public float MaxImpulseValue = float.PositiveInfinity;
    public float Reaction = 0f;
    public float Impulse = 0f;

    public ConstraintVariables()
    {
    }

    #region Helper Functions

    public static void SetReaction(Span<ConstraintVariables> vars, Vector3 r)
    {
        vars[0].Reaction = r.X;
        vars[1].Reaction = r.Y;
        vars[2].Reaction = r.Z;
    }

    public static void SetReaction(Span<ConstraintVariables> vars, float x, float y)
    {
        vars[0].Reaction = x;
        vars[1].Reaction = y;
    }

    public static void SetImpulse(Span<ConstraintVariables> vars, Vector3 i)
    {
        vars[0].Impulse = i.X;
        vars[1].Impulse = i.Y;
        vars[2].Impulse = i.Z;
    }

    public static void SetMinImpulses(Span<ConstraintVariables> vars, Vector3 mn)
    {
        vars[0].MinImpulseValue = mn.X;
        vars[1].MinImpulseValue = mn.Y;
        vars[2].MinImpulseValue = mn.Z;
    }

    public static void SetMinImpulses(Span<ConstraintVariables> vars, float mn)
    {
        vars[0].MinImpulseValue = mn;
        vars[1].MinImpulseValue = mn;
        vars[2].MinImpulseValue = mn;
    }

    public static void SetMaxImpulses(Span<ConstraintVariables> vars, Vector3 mx)
    {
        vars[0].MaxImpulseValue = mx.X;
        vars[1].MaxImpulseValue = mx.Y;
        vars[2].MaxImpulseValue = mx.Z;
    }

    public static void SetMaxImpulses(Span<ConstraintVariables> vars, float mx)
    {
        vars[0].MaxImpulseValue = mx;
        vars[1].MaxImpulseValue = mx;
        vars[2].MaxImpulseValue = mx;
    }

    #endregion

    public struct MovingRegression()
    {
        public float Confidence = 0.0f;
        public float LastPoint = 0.0f;
        public float LastTangent = 0.0f;
        public float LastCurvature = 0.0f;

        public float TestFitNextDataPointSecondOrder(float y)
        {
            var predicted = LastPoint + LastTangent + LastCurvature;
            var denom = Math.Max(Math.Abs(y), Math.Abs(predicted)) + 0.00001f;
            return Confidence * Math.Abs(y - predicted) / denom;
        }

        public float TestFitNextDataPointZeroOrder(float y)
        {
            var denom = Math.Max(Math.Abs(y), Math.Abs(LastPoint)) + 0.00001f;
            return Confidence * Math.Abs(y - LastPoint) / denom;
        }

        public void AddDataPoint(float y)
        {
            var newTangent = y - LastPoint;
            var newCurvature = newTangent - LastTangent;
            LastCurvature = newCurvature;
            LastTangent = newTangent;
            LastPoint = y;
            Confidence += 0.1f * (1.0f - Confidence);
        }
    }

    public struct ConstraintCache()
    {
        public float VelocityImpulse = 0.0f;
        public float VelocityReaction = 0.0f;
        public float VelocitySor = 1.9f; // Successive Over-Relaxation
        public float VelocityCacheDamping = 1.0f;
        public float PositionImpulse = 0.0f;
        public float PositionReaction = 0.0f;
        public float PositionSor = 1.9f;
        public float PositionCacheDamping = 1.0f;

        public MovingRegression VelocityImpulseRegression;
        public MovingRegression PositionImpulseRegression;

        public void ReadCache(ref ConstraintVariables velStage, ref ConstraintVariables posStage, ref float sorVel,
            ref float sorPos)
        {
            velStage.Impulse = VelocityImpulse;
            velStage.Reaction = VelocityReaction;
            sorVel = VelocitySor;
            posStage.Impulse = PositionImpulse;
            posStage.Reaction = PositionReaction;
            sorPos = PositionSor;
        }

        public void Cache(ref ConstraintVariables velStage, ref ConstraintVariables posStage, float sorVel,
            float sorPos, bool isCollision, SolverConfig config)
        {
            var velFit = VelocityImpulseRegression.TestFitNextDataPointSecondOrder(velStage.Impulse);
            var posFit = PositionImpulseRegression.TestFitNextDataPointSecondOrder(posStage.Impulse);

            // Use the fits to determine how much
            VelocityCacheDamping =
                calculateCacheDampingFactor(velFit, VelocityCacheDamping, config.CacheVStageModulation);
            PositionCacheDamping =
                calculateCacheDampingFactor(posFit, PositionCacheDamping, config.CachePStateModulation);

            if (!isCollision)
            {
                VelocitySor = recomputeSor(sorVel, velFit, config.SorConstraintsModulation);
                PositionSor = recomputeSor(sorPos, posFit, config.SorConstraintsModulation);
            }
            else
            {
                var vf = VelocityImpulseRegression.TestFitNextDataPointZeroOrder(velStage.Impulse);
                var pf = PositionImpulseRegression.TestFitNextDataPointZeroOrder(posStage.Impulse);
                VelocitySor = recomputeSor(sorVel, vf, config.SorCollisionsModulation);
                PositionSor = recomputeSor(sorPos, pf, config.SorCollisionsModulation);
            }

            VelocityImpulse = VelocityCacheDamping * velStage.Impulse;
            VelocityReaction = velStage.Reaction;
            PositionImpulse = PositionCacheDamping * posStage.Impulse;
            PositionReaction = posStage.Reaction;

            VelocityImpulseRegression.AddDataPoint(VelocityImpulse);
            PositionImpulseRegression.AddDataPoint(PositionImpulse);
        }
    }

    private static float calculateCacheDampingFactor(float fit, float oldFactor, in SolverConfig.ModulationParams cfg)
    {
        var t = SolverUtils.ReverseLerp(fit, cfg.ThresholdMin, cfg.ThresholdMax);

        // Determine how much we should apply SOR based on how good our fit is.
        var factor = t * cfg.ConservativeValue + (1.0f - t) * cfg.AggressiveValue;

        var easing = factor < oldFactor ? cfg.EasingDownToConservative : cfg.EasingUpToAggressive;

        return easing * factor + (1.0f - easing) * oldFactor;
    }

    private static float recomputeSor(float sor, float relError, in SolverConfig.ModulationParams cfg)
    {
        var newSor = cfg.AggressiveValue;

        if (relError > cfg.ThresholdMin)
        {
            var t = SolverUtils.ReverseLerp(relError, cfg.ThresholdMin, cfg.ThresholdMax);
            newSor = SolverUtils.Lerp(t, cfg.ConservativeValue, cfg.AggressiveValue);
        }

        if (newSor > sor)
            sor = cfg.EasingUpToAggressive * newSor + (1.0f - cfg.EasingUpToAggressive) * sor;
        else
            sor = cfg.EasingDownToConservative * newSor + (1.0f - cfg.EasingDownToConservative) * sor;

        return sor;
    }
}
