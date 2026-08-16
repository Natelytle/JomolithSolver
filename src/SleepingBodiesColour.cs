using Godot;

namespace JomolithSolver;

public partial class SleepingBodiesColour : MeshInstance3D
{
    private JomolithRigidBody? parent;
    private Color activeColor = Colors.Aquamarine;
    private Color sleepColour = Colors.IndianRed;

    private StandardMaterial3D mat = new();

    public override void _Ready()
    {
        parent = GetParent<JomolithRigidBody>();
        SetMaterialOverride(mat);
        mat.AlbedoColor = activeColor;
    }

    // public override void _PhysicsProcess(double delta)
    // {
    //     mat.AlbedoColor = parent?.Sleeping ?? false ? sleepColour : activeColor;
    // }
}
