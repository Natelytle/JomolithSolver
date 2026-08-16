using Godot;

namespace JomolithSolver;

public partial class CameraController : Camera3D
{
    private bool rightClick;

    public override void _Input(InputEvent @event)
    {
        if (rightClick)
        {
            if (@event is InputEventMouseMotion motion)
            {
                float rotX = float.Clamp(GlobalRotationDegrees.X - (motion.Relative.Y * 0.24f), -89f, 89f);
                float rotY = GlobalRotationDegrees.Y - (motion.Relative.X * 0.3f);

                GlobalRotationDegrees = new Vector3(rotX, rotY, GlobalRotationDegrees.Z);
            }
        }
    }

    public override void _Process(double delta)
    {
        if (Input.IsActionJustPressed("right_click"))
        {
            rightClick = true;
        }
        else if (Input.IsActionJustReleased("right_click"))
        {
            rightClick = false;
        }

        Vector2 direction = Input.GetVector("left", "right", "forward", "backward");
        float goingUp = Input.IsActionPressed("up") ? 1 : 0;
        float goingDown = Input.IsActionPressed("down") ? -1 : 0;

        Vector3 direction3D = new Vector3(direction.X, goingUp + goingDown, direction.Y);

        GlobalPosition += GlobalBasis * (40 * direction3D * (float)delta);
    }
}
