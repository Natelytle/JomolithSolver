namespace JomolithSolver.Solver;

public struct BodyPairIndices(int first, int second)
{
    public int First = first;
    public int Second = second;

    public BodyPairIndices() : this(0, 0)
    {
    }
}
