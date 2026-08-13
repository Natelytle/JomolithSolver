using System;
using System.Collections.Generic;
using System.Linq;
using JomolithSolver.Solver.Constraints;

namespace JomolithSolver.Solver;

public class ContactManager
{
    private readonly Dictionary<PairKey, ContactManifold> manifolds = new();

    public void Update(
        IReadOnlyDictionary<PairKey, List<ContactPoint>> freshByPair,
        IReadOnlyDictionary<PairKey, (Body A, Body B)> bodyLookup)
    {
        // update/prune existing manifolds — mirrors the erase-while-iterating
        // pattern from the C++, done safely via a snapshot of keys
        foreach (var key in manifolds.Keys.ToList())
        {
            var manifold = manifolds[key];

            if (freshByPair.TryGetValue(key, out var fresh))
            {
                manifold.Update(fresh);
            }
            else
            {
                manifold.Update(Array.Empty<ContactPoint>());
                if (manifold.Collisions.Count == 0) manifolds.Remove(key);
            }
        }

        // create manifolds for pairs that are new this frame
        foreach (var (key, points) in freshByPair)
        {
            if (manifolds.ContainsKey(key)) continue;
            if (!bodyLookup.TryGetValue(key, out var bodies)) continue;

            var m = new ContactManifold(bodies.A, bodies.B);
            m.Update(points);
            manifolds[key] = m;
        }
    }

    public bool IsGrounded(long rootUid, float upThreshold = 0.7f)
    {
        foreach (var m in manifolds.Values)
        {
            if (m.Collisions.Count == 0) continue;

            var isA = m.RootA.Uid == rootUid;
            var isB = m.RootB.Uid == rootUid;
            if (!isA && !isB) continue;

            foreach (var c in m.Collisions)
            {
                var n = c.Normal;
                var ny = isA ? -n.Y : n.Y;
                if (ny > upThreshold) return true;
            }
        }

        return false;
    }

    public bool IsBodyInContact(long rootUid)
    {
        foreach (var m in manifolds.Values)
        {
            if (m.Collisions.Count == 0) continue;
            if (m.RootA.Uid == rootUid || m.RootB.Uid == rootUid) return true;
        }

        return false;
    }

    public int GatherConstraints(
        List<Constraint> outConstraints,
        List<BodyPairIndices> outPairs,
        List<byte> outDims,
        IReadOnlyDictionary<long, int> rootUidToSolverIndex)
    {
        var added = 0;
        foreach (var m in manifolds.Values)
        {
            if (m.Collisions.Count == 0) continue;

            if (!rootUidToSolverIndex.TryGetValue(m.RootA.Uid, out var idxA)) continue;
            if (!rootUidToSolverIndex.TryGetValue(m.RootB.Uid, out var idxB)) continue;

            foreach (var c in m.Collisions)
            {
                outConstraints.Add(c);
                outPairs.Add(new BodyPairIndices(idxA, idxB));
                outDims.Add(3); // collisions are always 3-DOF
                added++;
            }
        }

        return added;
    }

    public void Clear()
    {
        manifolds.Clear();
    }

    public ContactManifold? FindManifold(long uidA, long uidB)
    {
        var key = new PairKey(uidA, uidB);
        return manifolds.TryGetValue(key, out var m) ? m : null;
    }

    public int TotalCollisionCount()
    {
        var total = 0;
        foreach (var m in manifolds.Values) total += m.Collisions.Count;
        return total;
    }

    public void UpdateFrictionForBody(long rootUid, float newFriction)
    {
        foreach (var m in manifolds.Values)
        {
            var isA = m.RootA.Uid == rootUid;
            var isB = m.RootB.Uid == rootUid;
            if (!isA && !isB) continue;

            var otherFriction = isA ? m.RootB.Friction : m.RootA.Friction;

            var c0 = Math.Clamp(newFriction, 0.0f, 2.0f);
            var c1 = Math.Clamp(otherFriction, 0.0f, 2.0f);
            var combined = (c0 <= 1.0f && c1 <= 1.0f) || (c0 >= 1.0f && c1 >= 1.0f)
                ? Math.Min(c0, c1)
                : c0 + c1 - 1.0f;

            foreach (var c in m.Collisions)
                c.Friction = combined;
        }
    }

    public readonly struct PairKey : IEquatable<PairKey>
    {
        public readonly long First;
        public readonly long Second;

        // self-sorting on construction — callers never need to sort manually,
        // unlike the C++ where every call site does (uidA < uidB) ? ... : ... by hand
        public PairKey(long a, long b)
        {
            if (a <= b)
            {
                First = a;
                Second = b;
            }
            else
            {
                First = b;
                Second = a;
            }
        }

        public bool Equals(PairKey other)
        {
            return First == other.First && Second == other.Second;
        }

        public override bool Equals(object? obj)
        {
            return obj is PairKey other && Equals(other);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(First, Second);
        }
    }
}
