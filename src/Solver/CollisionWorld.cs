using System;
using System.Collections.Generic;
using System.Numerics;
using BulletSharp;
using Vector3 = System.Numerics.Vector3;

namespace JomolithSolver.Solver;

public class CollisionWorld : IDisposable
{
    private const float BulletCollisionMargin = 0.045f;
    private readonly DbvtBroadphase broadphase;

    private readonly DefaultCollisionConfiguration config;
    private readonly CollisionDispatcher dispatcher;

    private readonly Dictionary<Body, BodyEntry> entries = new();
    private readonly BulletSharp.CollisionWorld world;

    public CollisionWorld()
    {
        config = new DefaultCollisionConfiguration();
        dispatcher = new CollisionDispatcher(config);
        broadphase = new DbvtBroadphase();
        world = new BulletSharp.CollisionWorld(dispatcher, broadphase, config);
    }

    public void Dispose()
    {
        foreach (var kv in entries)
        {
            world.RemoveCollisionObject(kv.Value.Obj);
            kv.Value.Obj.Dispose();
            kv.Value.Shape?.Dispose();
        }

        entries.Clear();

        world.Dispose();
        broadphase.Dispose();
        dispatcher.Dispose();
        config.Dispose();
    }

    private static CollisionShape? BuildShape(Body body)
    {
        var size = body.Size;

        switch (body.Shape)
        {
            case Body.ShapeType.Box:
            {
                var s = new BoxShape(size * 0.5f);
                s.Margin = BulletCollisionMargin;
                return s;
            }
            case Body.ShapeType.Sphere:
                // no margin, matches your original — Roblox spheres don't use one
                return new SphereShape(body.Size.X * 0.5f);

            case Body.ShapeType.Cylinder:
            {
                var s = new CylinderShapeX(size * 0.5f);
                s.Margin = BulletCollisionMargin;
                return s;
            }
            case Body.ShapeType.Wedge:
                return BuildWedgeHull(body.Size, false);

            case Body.ShapeType.CornerWedge:
                return BuildWedgeHull(body.Size, true);

            default:
                return null;
        }
    }

    private static ConvexHullShape BuildWedgeHull(Vector3 size, bool cornerWedge)
    {
        var x = size.X * 0.5f - BulletCollisionMargin;
        var y = size.Y * 0.5f - BulletCollisionMargin;
        var z = size.Z * 0.5f - BulletCollisionMargin;

        var hull = new ConvexHullShape();

        if (!cornerWedge)
        {
            hull.AddPoint(new Vector3(x, y, -z), false);
            hull.AddPoint(new Vector3(x, -y, -z), false);
            hull.AddPoint(new Vector3(x, -y, z), false);
            hull.AddPoint(new Vector3(-x, y, -z), false);
            hull.AddPoint(new Vector3(-x, -y, -z), false);
            hull.AddPoint(new Vector3(-x, -y, z));
        }
        else
        {
            hull.AddPoint(new Vector3(-x, -y, -z), false);
            hull.AddPoint(new Vector3(-x, -y, z), false);
            hull.AddPoint(new Vector3(x, y, -z), false);
            hull.AddPoint(new Vector3(x, -y, -z), false);
            hull.AddPoint(new Vector3(x, -y, z));
        }

        hull.Margin = BulletCollisionMargin;
        return hull;
    }

    public void AddBody(Body body)
    {
        if (entries.ContainsKey(body)) return;
        if (!body.CanCollide) return;
        if (body.Shape == Body.ShapeType.None) return;

        var e = new BodyEntry { Obj = new CollisionObject() };
        e.Shape = BuildShape(body);
        e.Obj.CollisionShape = e.Shape;

        e.Obj.CollisionFlags |= body.IsStatic
            ? CollisionFlags.StaticObject
            : CollisionFlags.KinematicObject;

        e.Obj.UserObject = body;

        world.AddCollisionObject(e.Obj);
        entries[body] = e;
    }

    public void RemoveBody(Body body)
    {
        if (!entries.TryGetValue(body, out var e)) return;
        world.RemoveCollisionObject(e.Obj);
        e.Obj.Dispose();
        e.Shape?.Dispose();
        entries.Remove(body);
    }

    public void UpdateBodyShape(Body body)
    {
        if (!entries.TryGetValue(body, out var e)) return;

        world.RemoveCollisionObject(e.Obj);
        e.Shape?.Dispose();

        e.Shape = BuildShape(body);
        e.Obj.CollisionShape = e.Shape;

        world.AddCollisionObject(e.Obj);
    }

    public void SyncTransforms(IEnumerable<Body> bodies)
    {
        foreach (var body in bodies)
        {
            if (!entries.TryGetValue(body, out var e)) continue;
            if (e.Obj.CollisionShape == null) continue;

            var cf = body.GetWorldCFrame();
            var tr = ToMatrixWithTransform(cf.Rotation, cf.Translation);
            e.Obj.WorldTransform = tr;
            e.Obj.InterpolationWorldTransform = tr;
        }
    }

    public void DetectCollisions()
    {
        world.PerformDiscreteCollisionDetection();
    }

    public void ExtractContacts(
        Dictionary<ContactManager.PairKey, List<ContactPoint>> outContacts,
        Dictionary<ContactManager.PairKey, (Body, Body)> outBodyLookup)
    {
        outContacts.Clear();
        outBodyLookup.Clear();

        var numManifolds = dispatcher.NumManifolds;
        for (var i = 0; i < numManifolds; i++)
        {
            var manifold = dispatcher.GetManifoldByIndexInternal(i);
            if (manifold == null || manifold.NumContacts == 0) continue;

            var obA = manifold.Body0;
            var obB = manifold.Body1;
            if (obA == null || obB == null) continue;

            var leafA = obA.UserObject as Body;
            var leafB = obB.UserObject as Body;
            if (leafA == null || leafB == null) continue;

            var rootA = leafA.GetRoot();
            var rootB = leafB.GetRoot();

            if (rootA == rootB) continue;
            if (rootA.IsStatic && rootB.IsStatic) continue;

            var uidA = rootA.Uid;
            var uidB = rootB.Uid;
            var key = uidA < uidB
                ? new ContactManager.PairKey(uidA, uidB)
                : new ContactManager.PairKey(uidB, uidA);

            outBodyLookup[key] = uidA < uidB ? (rootA, rootB) : (rootB, rootA);

            var c0 = Math.Clamp(leafA.Friction, 0.0f, 2.0f);
            var c1 = Math.Clamp(leafB.Friction, 0.0f, 2.0f);
            var combinedFriction = (c0 <= 1.0f && c1 <= 1.0f) || (c0 >= 1.0f && c1 >= 1.0f)
                ? Math.Min(c0, c1)
                : c0 + c1 - 1.0f;
            var combinedRestitution = 0.5f * (leafA.Restitution + leafB.Restitution);

            var swapAB = uidA > uidB;

            if (!outContacts.TryGetValue(key, out var points))
            {
                points = new List<ContactPoint>();
                outContacts[key] = points;
            }

            for (var c = 0; c < manifold.NumContacts; c++)
            {
                var mp = manifold.GetContactPoint(c);
                var depth = mp.Distance1;
                if (depth > 0.0f) continue;

                var normalB = mp.NormalWorldOnB;

                Vector3 normal;
                Vector3 posOnA;
                if (swapAB)
                {
                    normal = normalB;
                    posOnA = mp.PositionWorldOnB;
                }
                else
                {
                    normal = -normalB;
                    posOnA = mp.PositionWorldOnA;
                }

                var nlen = normal.Length();
                if (nlen < 0.9f) continue;
                normal /= nlen;

                points.Add(new ContactPoint
                {
                    PositionOnA = posOnA,
                    Normal = normal,
                    Depth = depth,
                    Friction = combinedFriction,
                    Restitution = combinedRestitution
                });

                // Console.WriteLine($"[contact] depth={depth:F5} normal={normal} posA={posOnA}");
            }

            if (points.Count == 0) outContacts.Remove(key);
        }
    }

    public bool RaycastClosest(Vector3 start, Vector3 end,
        out RayHit outHit, IReadOnlyList<Body>? ignoreBodies = null)
    {
        outHit = default;

        var ignoreSet = new HashSet<CollisionObject>();
        if (ignoreBodies != null)
            foreach (var b in ignoreBodies)
                if (entries.TryGetValue(b, out var e))
                    ignoreSet.Add(e.Obj);

        var from = start;
        var to = end;
        var callback = new FilteredRayResultCallback(ref from, ref to, ignoreSet);
        world.RayTest(from, to, callback);

        if (!callback.HasHit) return false;

        outHit.Point = callback.HitPointWorld;
        outHit.Normal = callback.HitNormalWorld;
        outHit.Body = callback.CollisionObject.UserObject as Body;
        return true;
    }

    public void SetBodyContactResponse(Body body, bool enabled)
    {
        if (!entries.TryGetValue(body, out var e)) return;
        if (enabled)
            e.Obj.CollisionFlags &= ~CollisionFlags.NoContactResponse;
        else
            e.Obj.CollisionFlags |= CollisionFlags.NoContactResponse;
    }

    public void SetBodyFriction(Body body, float friction)
    {
        if (!entries.TryGetValue(body, out var e)) return;
        e.Obj.Friction = friction;
        e.Obj.RollingFriction = 0.0f;
    }

    public List<Body> GetBodiesInAABB(Vector3 min, Vector3 max,
        IReadOnlyList<Body>? ignoreBodies = null)
    {
        var result = new List<Body>();
        var ignoreSet = ignoreBodies != null ? new HashSet<Body>(ignoreBodies) : null;

        foreach (var kv in entries)
        {
            var body = kv.Key;
            var e = kv.Value;
            if (e.Obj.CollisionShape == null) continue;
            if (ignoreSet != null && ignoreSet.Contains(body)) continue;

            e.Obj.CollisionShape.GetAabb(e.Obj.WorldTransform, out var aabbMin, out var aabbMax);

            if (aabbMin.X <= max.X && aabbMax.X >= min.X &&
                aabbMin.Y <= max.Y && aabbMax.Y >= min.Y &&
                aabbMin.Z <= max.Z && aabbMax.Z >= min.Z)
                result.Add(body);
        }

        return result;
    }

    public void BeginBatchLoad()
    {
        world.PairCache.SetOverlapFilterCallback(null);
    }

    public void EndBatchLoad()
    {
        world.Broadphase.ResetPool(dispatcher);
        world.PairCache.SetOverlapFilterCallback(null);
    }

    // -------- conversion helpers --------

    private static Matrix4x4 ToMatrixWithTransform(Matrix4x4 rotation, Vector3 translation)
    {
        // build directly, mirroring your mat3ToBtTransform's column layout
        var m = Matrix4x4.Identity;
        m.M11 = rotation.M11;
        m.M12 = rotation.M21;
        m.M13 = rotation.M31;
        m.M21 = rotation.M12;
        m.M22 = rotation.M22;
        m.M23 = rotation.M32;
        m.M31 = rotation.M13;
        m.M32 = rotation.M23;
        m.M33 = rotation.M33;
        m.M41 = translation.X;
        m.M42 = translation.Y;
        m.M43 = translation.Z;
        return m;
    }

    private class BodyEntry
    {
        public required CollisionObject Obj;
        public CollisionShape? Shape; // one field covers all shape types now
    }

    public struct RayHit
    {
        public Vector3 Point;
        public Vector3 Normal;
        public Body? Body;
    }

    private class FilteredRayResultCallback : ClosestRayResultCallback
    {
        private readonly HashSet<CollisionObject> _ignore;

        public FilteredRayResultCallback(ref Vector3 from, ref Vector3 to, HashSet<CollisionObject> ignore)
            : base(ref from, ref to)
        {
            _ignore = ignore;
        }

        public override float AddSingleResult(ref LocalRayResult rayResult, bool normalInWorldSpace)
        {
            if (_ignore.Contains(rayResult.CollisionObject))
                return 1.0f; // tell Bullet to keep looking

            return base.AddSingleResult(ref rayResult, normalInWorldSpace);
        }
    }
}
