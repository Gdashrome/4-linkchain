using UnityEngine;

[System.Serializable]
public struct IKRawData
{
    public int testNum;
    public int iteration;
    public float iterationTime;
    public float elapsedIterationTime;
    public float iterationError;

    public IKRawData(int test, int iter, float iterTime, float elapsedIterTime, float iterErr)
    {
        testNum = test;
        iteration = iter;
        iterationTime = iterTime;
        elapsedIterationTime = elapsedIterTime;
        iterationError = iterErr;
    }
}