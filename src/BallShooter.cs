using System;
using Godot;

namespace JomolithSolver;

public partial class BallShooter : Node
{
    [Export] private Camera3D camera { get; set; }

    public override void _Input(InputEvent @event)
    {
        if (@event is InputEventMouseButton mouseButton)
        {
            if (mouseButton.ButtonIndex == MouseButton.Left && mouseButton.Pressed)
            {
                Vector3 worldPos = camera.ProjectPosition(mouseButton.Position, 0);
                Vector3 direction = camera.ProjectRayNormal(mouseButton.Position);

                JomolithRigidBody BALL = new JomolithRigidBody
                {
                    Shape = JomolithRigidBody.ShapeType.Ball,
                    Size = new Vector3(2, 2, 2),
                    Density = 0.7f
                };

                BALL.AddChild(new SleepingBodiesColour
                {
                    Mesh = new SphereMesh()
                    {
                        Radius = 1,
                        Height = 2
                    }
                });

                GetParent().AddChild(BALL);

                BALL.GlobalPosition = worldPos;
                BALL.GlobalBasis *= camera.GlobalBasis;
                BALL.GlobalRotate(camera.Basis.Y, float.Pi / 2);

                BALL.AccumulateImpulse(direction * 300 * BALL.Mass);
            }
        }
    }
}
