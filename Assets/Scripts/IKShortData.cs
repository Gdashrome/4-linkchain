using UnityEngine;

[System.Serializable]
public struct IKShortData
{
    public int testNum;
    public int iterations;
    public float averageIterationTime;
    public float totalIterationTime;
    public float initialError;
    public float finalError;
    public Vector2 targetPosition;
    public float targetDistanceFromBase;

    public IKShortData(int test, int iters, float avgIterTime, float totalIterTime, float initialErr, float finalErr, Vector2 targetPos, float targetDistFromBase)
    {
        testNum = test;
        iterations = iters;
        averageIterationTime = avgIterTime;
        totalIterationTime = totalIterTime;
        initialError = initialErr;
        finalError = finalErr;
        targetPosition = targetPos;
        targetDistanceFromBase = targetDistFromBase;
    }
}