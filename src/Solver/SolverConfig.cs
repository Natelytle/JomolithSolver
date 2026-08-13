namespace JomolithSolver.Solver;

public struct SolverConfig()
{
    public int PgsIterations = 20;

    public float CollisionRestitutionThreshold = 1.0f;
    public float CollisionPenetrationMargin = 0.05f;
    public float CollisionPenetrationMarginMax = 0.05f;
    public float CollisionPenetrationMarginMin = 0.0001f;
    public float CollisionPenetrationMarginMaxBumpProportions = 0.05f;
    public float CollisionPenetrationResolutionDamping = 0.7f;
    public float CollisionPenetrationVelocityForMinMargin = 20.0f;
    public float CollisionFrictionStaticToDynamicThreshold = 1.0f;
    public float CollisionFrictionStaticScale = 1.0f;
    public float CollisionFrictionDynamicScale = 1.0f;

    public float Align2AxesFrictionConstant = 0.02f;
    public float Align2AxesPositionStageFrictionConstant = 0.01f;
    public float Align2AxesMaxCorrectiveAngle = 3.0f;
    public float Align2AxesCorrectionDamping = 1.0f;

    public float BallInSocketMaxCorrectionByStabilization = 0.3f;
    public float BallInSocketCorrectionDamping = 1.0f;

    public struct ModulationParams
    {
        public float ThresholdMax;
        public float ThresholdMin;
        public float AggressiveValue;
        public float ConservativeValue;
        public float EasingUpToAggressive;
        public float EasingDownToConservative;
    }

    public ModulationParams SorConstraintsModulation = new()
    {
        ThresholdMax = 1.0f, ThresholdMin = 0.01f, AggressiveValue = 1.5f, ConservativeValue = 0.75f,
        EasingUpToAggressive = 0.0001f, EasingDownToConservative = 0.2f
    };

    public ModulationParams SorCollisionsModulation = new()
    {
        ThresholdMax = 0.05f, ThresholdMin = 0.01f, AggressiveValue = 1.9f, ConservativeValue = 1.0f,
        EasingUpToAggressive = 0.0001f, EasingDownToConservative = 0.2f
    };

    public ModulationParams CacheVStageModulation = new()
    {
        ThresholdMax = 0.5f, ThresholdMin = 2.0f, AggressiveValue = 0.93f, ConservativeValue = 0.99f,
        EasingUpToAggressive = 0.01f, EasingDownToConservative = 0.0002f
    };

    public ModulationParams CachePStateModulation = new()
    {
        ThresholdMax = 0.5f, ThresholdMin = 1.5f, AggressiveValue = 0.70f, ConservativeValue = 0.99f,
        EasingUpToAggressive = 0.01f, EasingDownToConservative = 0.0002f
    };

    public float StabilizationMassReductionPower = 0.3f;
    public float StabilizationInertiaScale = 0.05f;

    public float VelCacheDamping = 1.0f;
    public float PosCacheDamping = 1.0f;
    public bool ConstraintCachingEnabled = true;

    public float AngularDamping = 0.911328f; // -ln(0.99621) / (1/240)
    public bool UpdateSimBodies = true;
    public bool IntegrateOnlyPositions = false;

    public bool BlockPGSEnabled = true;
    public bool VelocityStageSOREnabled = true;
    public bool PositionStateSOREnabled = true;
    public bool VirtualMassesEnabled = true;
    public bool UseSimIslands = false;

    public bool InconsistentConstraintDetectorEnabled = false;
    public int InconsistentConstraintMaxIterations = 0;
    public float InconsistentConstraintBallInSocketResidualThreshold = 0.02f;
    public float InconsistentConstraintDeltaThreshold = 0.0001f;
    public float InconsistentConstraintAlign2AxesThreshold = 0.003f;
    public float InconsistentConstraintCollisionThreshold = 0.02f;
    public float InconsistentConstraintCollisionBaseThreshold = 0.001f;
    public bool PrintConvergenceDiagnostics = false;
}
