using UnityEngine;

[System.Serializable]
public struct IKShortData
{
    public int testNum;
    public int iterations;
    public float averageIterationTime;
    public float totalIterationTime;
    public float finalError;

    public IKShortData(int test, int iters, float avgIterTime, float totalIterTime, float finalErr)
    {
        testNum = test;
        iterations = iters;
        averageIterationTime = avgIterTime;
        totalIterationTime = totalIterTime;
        finalError = finalErr;
    }
}