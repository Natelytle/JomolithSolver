using Godot;
using JomolithSolver.Solver;
using JomolithSolver.Utils;

namespace JomolithSolver;

[GlobalClass]
public partial class JomolithRigidBody : Node3D
{
    public enum ShapeType
    {
        Box,
        Ball,
        Cylinder,
        Wedge,
        CornerWedge
    }

    public Body SolverBody { get; } = new();

    [Export]
    public JomolithRigidBody? Parent
    {
        get;
        set
        {
            SolverBody.Detach();
            field = value;
            AttachToParent(value);
        }
    } = null;

    private void AttachToParent(JomolithRigidBody? parent)
    {
        if (parent is not null)
        {
            var worldCf = new CoordinateFrame(GlobalBasis.ToMatrix(), GlobalPosition.ToNumerics());
            var parentCf = new CoordinateFrame(parent.GlobalBasis.ToMatrix(), parent.GlobalPosition.ToNumerics());

            var parentBody = parent.SolverBody;
            var localCfChild = parentCf.Inverse() * worldCf;

            parentBody.WeldChild(SolverBody, localCfChild);
        }
    }

    // Exported properties are kind of roundabout, this is to ensure that the default values display properly in the godot editor.
    #region Exported Properties

    [Export]
    public ShapeType Shape
    {
        get;
        set
        {
            field = value;
            SolverBody.SetShape((Body.ShapeType)value);
        }
    } = ShapeType.Box;

    [Export]
    public Vector3 Size
    {
        get;
        set
        {
            field = value;
            SolverBody.SetSize(value.ToNumerics());
        }
    } = new(2, 1, 4);

    [Export]
    public float Density
    {
        get;
        set
        {
            field = value;
            SolverBody.SetDensity(value);
        }
    } = 0.7f;

    [Export]
    public float Restitution
    {
        get;
        set
        {
            field = value;
            SolverBody.Restitution = value;
        }
    } = 0.5f;

    [Export]
    public float Friction
    {
        get;
        set
        {
            field = value;
            SolverBody.Friction = value;
        }
    } = 0.3f;

    [Export]
    public bool Anchored
    {
        get;
        set
        {
            field = value;
            SolverBody.IsStatic = value;
        }
    } = false;

    #endregion

    public float Mass => SolverBody.Mass;

    public Vector3 LinearVelocity
    {
        get => SolverBody.LinearVelocity.ToGodot();
        set => SolverBody.LinearVelocity = value.ToNumerics();
    }

    public Vector3 AngularVelocity
    {
        get => SolverBody.AngularVelocity.ToGodot();
        set => SolverBody.AngularVelocity = value.ToNumerics();
    }

    public sealed override void _Ready()
    {
        ((JomolithSolverWorld)GetTree().GetFirstNodeInGroup("SolverWorld")).RegisterBody(this);

        // Initialize properties after godot sets the field values
        SolverBody.SetShape((Body.ShapeType)Shape);
        SolverBody.SetSize(Size.ToNumerics());
        SolverBody.SetDensity(Density);
        SolverBody.Friction = Friction;
        SolverBody.Restitution = Restitution;
        SolverBody.IsStatic = Anchored;
        AttachToParent(Parent);

        _ReadyInternal();
    }

    protected virtual void _ReadyInternal() { }

    public sealed override void _ExitTree()
    {
        ((JomolithSolverWorld)GetTree().GetFirstNodeInGroup("SolverWorld")).UnregisterBody(this);

        _ExitTreeInternal();
    }

    protected virtual void _ExitTreeInternal() { }

    public virtual void _PrePhysicsProcess(double delta) { }

    public void AccumulateForce(in Vector3 f) => SolverBody.AccumulateForce(f.ToNumerics());

    public void AccumulateTorque(in Vector3 t) => SolverBody.AccumulateForce(t.ToNumerics());

    public void AccumulateForceAtPoint(in Vector3 f, in Vector3 worldPoint) => SolverBody.AccumulateForceAtPoint(f.ToNumerics(), worldPoint.ToNumerics());

    public void AccumulateImpulse(in Vector3 i) => SolverBody.AccumulateImpulse(i.ToNumerics());

    public void AccumulateImpulseAtPoint(in Vector3 i, in Vector3 worldPoint) => SolverBody.AccumulateImpulseAtPoint(i.ToNumerics(), worldPoint.ToNumerics());

    public void AccumulateRotationalImpulse(in Vector3 ri) => SolverBody.AccumulateRotationalImpulse(ri.ToNumerics());
}
