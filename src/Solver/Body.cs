using System.Collections.Generic;
using System.Numerics;
using JomolithSolver.Solver.Utils;

namespace JomolithSolver.Solver;

public class Body(long uid = 0)
{
    public enum ShapeType
    {
        Box,
        Sphere,
        Cylinder,
        Wedge,
        CornerWedge,
        None,
    }

    private readonly List<Body> children = [];

    private Vector3 branchCofmOffsetLocal;

    private bool branchDirty = true;
    private Matrix4x4 branchInertiaLocal;

    public Vector3 ExternalForce { get; private set; } = Vector3.Zero;
    public Vector3 ExternalImpulse { get; private set; } = Vector3.Zero;
    public Vector3 ExternalRotationalImpulse { get; private set; } = Vector3.Zero;
    public Vector3 ExternalTorque { get; private set; } = Vector3.Zero;

    private CoordinateFrame localCFrame = new();
    private CoordinateFrame worldCFrame = new();
    private float branchMass;

    public long Uid { get; set; } = uid;

    // These properties need branch recomputation
    public ShapeType Shape { get; private set; } = ShapeType.Box;
    public float Mass { get; private set; } = 1.0f;
    public float Density => MassProperties.DensityOf(Shape, Mass, Size);
    public Vector3 Size { get; private set; } = Vector3.One;
    public Matrix4x4 InertiaBody { get; private set; } = Matrix4x4.Identity;

    // These properties apply instantly
    public float Friction { get; set; } = 0.3f;
    public float Restitution { get; set; } = 0.5f;

    public bool IsStatic
    {
        get;
        set
        {
            field = value;
            GetRoot().MarkBranchDirty();
        }
    }

    public bool CanCollide { get; set; } = true;

    public Vector3 LinearVelocity { get; set; } = Vector3.Zero;
    public Vector3 AngularVelocity { get; set; } = Vector3.Zero;

    public Body? Parent { get; private set; }

    public IReadOnlyList<Body> Children => children;

    public bool IsRoot()
    {
        return Parent == null;
    }

    public void WeldChild(Body child, in CoordinateFrame newChildCFrame)
    {
        if (child.Parent is not null)
            child.Detach();

        child.Parent = this;
        child.localCFrame = newChildCFrame;
        children.Add(child);

        var r = GetRoot();

        r.MarkBranchDirty();
    }

    public void Detach()
    {
        if (Parent is null) return;
        Parent.children.Remove(this);
        Parent.GetRoot().MarkBranchDirty();
        Parent = null;
        MarkBranchDirty();
    }

    public Body GetRoot()
    {
        return IsRoot() ? this : Parent!.GetRoot();
    }

    public void MarkBranchDirty()
    {
        var r = GetRoot();

        r.branchDirty = true;
    }

    public void SetProperties(ShapeType shape, Vector3 size, MassProperties properties)
    {
        Mass = properties.Mass;
        Size = size;
        Shape = shape;
        InertiaBody = properties.InertiaBody;
        GetRoot().MarkBranchDirty();
    }

    public void SetShape(ShapeType shape) => SetProperties(shape, Size, MassProperties.ForShape(Shape, Density, Size));

    public void SetDensity(float density) => SetProperties(Shape, Size, MassProperties.ForShape(Shape, density, Size));

    public void SetSize(Vector3 size) => SetProperties(Shape, size, MassProperties.ForShape(Shape, Density, size));

    public void AccumulateForce(in Vector3 f)
    {
        if (IsStatic) return;
        GetRoot().ExternalForce += f;
    }

    public void AccumulateTorque(in Vector3 t)
    {
        if (IsStatic) return;
        GetRoot().ExternalTorque += t;
    }

    public void AccumulateForceAtPoint(in Vector3 f, in Vector3 worldPoint)
    {
        if (IsStatic) return;
        var r = GetRoot();
        r.ExternalForce += f;
        var rel = worldPoint - r.GetWorldCFrame().Translation;
        r.ExternalTorque += Vector3.Cross(rel, f);
    }

    public void AccumulateImpulse(in Vector3 i)
    {
        if (IsStatic) return;
        GetRoot().ExternalImpulse += i;
    }

    public void AccumulateImpulseAtPoint(in Vector3 i, in Vector3 worldPoint)
    {
        if (IsStatic) return;
        var r = GetRoot();
        r.ExternalImpulse += i;
        var rel = worldPoint - r.GetWorldCFrame().Translation;
        r.ExternalRotationalImpulse += Vector3.Cross(rel, i);
    }

    public void AccumulateRotationalImpulse(in Vector3 ri)
    {
        if (IsStatic) return;
        GetRoot().ExternalRotationalImpulse += ri;
    }

    public void ClearAccumulators()
    {
        var r = GetRoot();
        r.ExternalForce = Vector3.Zero;
        r.ExternalTorque = Vector3.Zero;
        r.ExternalImpulse = Vector3.Zero;
        r.ExternalRotationalImpulse = Vector3.Zero;
    }

    public Vector3 GetBranchCofmWorld()
    {
        var r = GetRoot();

        if (r.branchDirty) r.RecomputeBranchProperties();

        return r.GetWorldCFrame().Translation + Vector3.TransformNormal(r.branchCofmOffsetLocal, r.GetWorldCFrame().Rotation);
    }

    public Matrix4x4 GetBranchInertiaWorldAtPoint(Vector3 worldPoint)
    {
        var r = GetRoot();

        if (r.branchDirty) r.RecomputeBranchProperties();

        var rot = r.GetWorldCFrame().Rotation;
        var iWorld = Matrix4x4.Transpose(rot) * r.branchInertiaLocal * rot;

        var cofmWorld = r.GetWorldCFrame().Translation + Vector3.TransformNormal(r.branchCofmOffsetLocal, Matrix4x4.Transpose(rot));
        var d = worldPoint - cofmWorld;
        var l = d.Length();

        var shift = BuildShiftMatrix(d);

        shift *= r.branchMass;
        return iWorld + shift;
    }

    public Vector3 GetBranchIBodyV3()
    {
        var r = GetRoot();

        if (r.branchDirty) r.RecomputeBranchProperties();

        return new Vector3(r.branchInertiaLocal[0][0], r.branchInertiaLocal[1][1], r.branchInertiaLocal[2][2]);
    }

    public float GetBranchMass()
    {
        var r = GetRoot();

        if (r.branchDirty) r.RecomputeBranchProperties();

        return r.branchMass;
    }

    public CoordinateFrame GetWorldCFrame()
    {
        if (Parent is null) return worldCFrame;

        return Parent.GetWorldCFrame() * localCFrame;
    }

    public void SetWorldCFrame(CoordinateFrame cf)
    {
        if (Parent is not null) return;
        worldCFrame = cf;
    }

    public void SetWorldCFrame(Matrix4x4 rotation, Vector3 translation)
    {
        if (Parent is not null) return;
        worldCFrame.Rotation = rotation;
        worldCFrame.Translation = translation;
    }

    public void RecomputeBranchProperties()
    {
        if (Parent is not null)
        {
            GetRoot().RecomputeBranchProperties();
            return;
        }

        if (!branchDirty) return;

        var totalMass = 0.0f;
        var cofmWorldAccumulated = Vector3.Zero;
        var inertiaWorldOrigin = new Matrix4x4();

        RecomputeBranchRecursive(ref totalMass, ref cofmWorldAccumulated, ref inertiaWorldOrigin);

        branchMass = totalMass;

        if (totalMass > 0.0f)
        {
            var cofmWorld = cofmWorldAccumulated / totalMass;
            branchCofmOffsetLocal = Vector3.TransformNormal(cofmWorld - GetWorldCFrame().Translation, Matrix4x4.Transpose(GetWorldCFrame().Rotation));

            var r = cofmWorld;

            var shift = BuildShiftMatrix(r);

            shift *= totalMass;
            var inertiaAboutCofm = inertiaWorldOrigin - shift;

            branchInertiaLocal =  GetWorldCFrame().Rotation * inertiaAboutCofm * Matrix4x4.Transpose(GetWorldCFrame().Rotation);
        }
        else
        {
            branchCofmOffsetLocal = Vector3.Zero;
            branchInertiaLocal = new Matrix4x4();
        }

        branchDirty = false;
    }

    private void RecomputeBranchRecursive(ref float outMass, ref Vector3 outCofmWorld, ref Matrix4x4 outInertiaWorld)
    {
        var myWorld = Parent is not null ? Parent.GetWorldCFrame() * localCFrame : worldCFrame;

        if (!IsStatic && Mass > 0.0f)
        {
            var myCofmWorld = myWorld.Translation;
            outMass += Mass;
            outCofmWorld += myCofmWorld * Mass;

            var rot = myWorld.Rotation;
            var iWorld = Matrix4x4.Transpose(rot) * InertiaBody * rot;
            var r = myCofmWorld;

            var shift = BuildShiftMatrix(r);

            shift *= Mass;
            outInertiaWorld += iWorld + shift;
        }

        foreach (var body in Children)
            body.RecomputeBranchRecursive(ref outMass, ref outCofmWorld, ref outInertiaWorld);
    }

    private static Matrix4x4 BuildShiftMatrix(Vector3 r)
    {
        float rDotR = Vector3.Dot(r, r);
        Matrix4x4 rOutR = OuterProduct(r, r);
        var shift = Matrix4x4.Identity * rDotR - rOutR;

        return shift;
    }

    private static Matrix4x4 OuterProduct(Vector3 first, Vector3 second)
    {
        float[,] result = new float[3, 3];

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result[i, j] += first[i] * second[j];
            }
        }

        return new Matrix4x4(
            result[0, 0], result[0, 1], result[0, 2], 0,
            result[1, 0], result[1, 1], result[1, 2], 0,
            result[2, 0], result[2, 1], result[2, 2], 0,
            0, 0, 0, 1
        );
    }
}
