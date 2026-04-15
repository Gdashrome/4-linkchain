using System.IO;
using System.Text;
using System.Collections.Generic;
using UnityEngine;

public static class CSVWriter
{
    public static void WriteRawData(string fileName, List<IKRawData> dataList)
    {
        StringBuilder sb = new StringBuilder();

        // Header
        sb.AppendLine("TestNum,Iteration,IterationTime,ElapsedTime,Error");

        // Rows
        foreach (var data in dataList)
        {
            sb.AppendLine($"{data.testNum},{data.iteration},{data.iterationTime:F12},{data.elapsedIterationTime:F12},{data.iterationError}");
        }

        WriteToFile(fileName, sb.ToString());
    }

    public static void WriteShortData(string fileName, List<IKShortData> dataList)
    {
        StringBuilder sb = new StringBuilder();

        // Header
        sb.AppendLine("TestNum,Iterations,AvgIterationTime,TotalTime,InitialError,FinalError,TargetX,TargetY,TargetDistanceFromBase");

        foreach (var data in dataList)
        {
            sb.AppendLine($"{data.testNum},{data.iterations},{data.averageIterationTime:F12},{data.totalIterationTime:F12},{data.initialError:F12},{data.finalError:F12},{data.targetPosition.x:F12},{data.targetPosition.y:F12},{data.targetDistanceFromBase:F12}");
        }

        WriteToFile(fileName, sb.ToString());
    }

    private static void WriteToFile(string fileName, string content)
    {
        string folderPath = Path.Combine(Application.dataPath, "CSV Data");
        string fullPath = Path.Combine(folderPath, fileName);

        File.WriteAllText(fullPath, content);

        Debug.Log("CSV written to: " + fullPath);
    }
}