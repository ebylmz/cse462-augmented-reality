using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using Accord.Math;

public class GeneratePointClouds : MonoBehaviour
{
    [SerializeField] private bool run;

    // Start is called before the first frame update
    void Start()
    {
        if (run) {
            // uncomment to create the specific point clouds
            // CreatePC25(); 
            // CreatePC20(); 
            // CreatePC30(); 
            CreatePC5(); 
            CreatePC7();
        }
    }    

    public void Generate(int pcloudSize, Accord.Math.Matrix3x3 R, Accord.Math.Vector3 T, Accord.Math.Matrix3x3 S, out List<Accord.Math.Vector3> pcloud1, out List<Accord.Math.Vector3> pcloud2)
    {
        pcloud1 = new List<Accord.Math.Vector3>();        
        pcloud2 = new List<Accord.Math.Vector3>();        

        int xyzMin = 0;
        int xyzMax = 15;

        // Create a randomly constructed point cloud for pcloud1
        System.Random rand = new System.Random(42);
        for (int i = 0; i < pcloudSize; ++i) {
            Accord.Math.Vector3 p = new Accord.Math.Vector3(
                (float)rand.Next(xyzMin, xyzMax),
                (float)rand.Next(xyzMin, xyzMax),
                (float)rand.Next(xyzMin, xyzMax)
            );

            pcloud1.Add(p);

            // Apply given transformation to produce the second point cloud
            pcloud2.Add(S * R * p + T); 
        }
    }

    public void Export(List<Accord.Math.Vector3> pointCloud, string exportFileName)
    {   
        string path = Path.Combine(Application.dataPath, "Resources", $"{exportFileName}.txt");

        try {
            using (StreamWriter writer = new StreamWriter(path)) {
                // Write the number of points as the first line
                writer.WriteLine(pointCloud.Count);

                // Write each point's coordinates (X, Y, Z) to the file
                foreach (Accord.Math.Vector3 point in pointCloud) {
                    writer.WriteLine($"{point.X:F2} {point.Y:F2} {point.Z:F2}");
                }
            }
            Debug.Log($"Point cloud successfully exported to {path}");
        }
        catch (System.Exception e) {
            Debug.LogError($"Error exporting point cloud: {e.Message}");
        }
    }

    // Fisher-Yates shuffle algorithm
    private void Shuffle<T>(List<T> list)
    {
        int n = list.Count;
        while (n > 1) {
            --n;
            int k = Random.Range(0, n + 1);
            T value = list[k];
            list[k] = list[n];
            list[n] = value;
        }
    }

    public void CreateExportPointCloud(int pcloudSize, bool shuffle, string pcloudName, Accord.Math.Matrix3x3 R, Accord.Math.Vector3 T, Accord.Math.Matrix3x3 S)
    {
        List<Accord.Math.Vector3> pcloud1, pcloud2;

        Generate(pcloudSize, R, T, S, out pcloud1, out pcloud2);

        // Randomize the list order
        if (shuffle) {
            Shuffle<Accord.Math.Vector3>(pcloud1);
            Shuffle<Accord.Math.Vector3>(pcloud2);
        }

        Export(pcloud1, $"{pcloudName}_1");
        Export(pcloud2, $"{pcloudName}_2");
    }

    private void CreatePC30() 
    {
        int pcloudSize = 30;
        string pcloudName = $"pc{pcloudSize}";
        Accord.Math.Vector3 T = new Accord.Math.Vector3(10, 10, 10);
        Accord.Math.Matrix3x3 R = Utility.CreateRotationMatrix(0, 90, 0);

        Accord.Math.Matrix3x3 S = Utility.ConstructAccordMatrix3x3(2);

        CreateExportPointCloud(pcloudSize, false, pcloudName, R, T, S);
    }

    private void CreatePC25() 
    {
        int pcloudSize = 25;
        string pcloudName = $"pc{pcloudSize}";
        Accord.Math.Vector3 T = new Accord.Math.Vector3(30, 40, 10);
        Accord.Math.Matrix3x3 R = Utility.CreateRotationMatrix(90, 0, 0);
        // No scaling
        Accord.Math.Matrix3x3 S = Utility.ConstructAccordMatrix3x3(1);

        CreateExportPointCloud(pcloudSize, true, pcloudName, R, T, S);
    }

    private void CreatePC20() 
    {
        int pcloudSize = 20;
        string pcloudName = $"pc{pcloudSize}";
        Accord.Math.Vector3 T = new Accord.Math.Vector3(30, 40, 10);
        Accord.Math.Matrix3x3 R = Utility.CreateRotationMatrix(45, 0, 0);
        Accord.Math.Matrix3x3 S = Utility.ConstructAccordMatrix3x3(
            2, 0, 0, 
            0, 2, 0, 
            0, 0, 1);

        CreateExportPointCloud(pcloudSize, true, pcloudName, R, T, S);
    }

    private void CreatePC7() 
    {
        int pcloudSize = 7;
        string pcloudName = $"pc{pcloudSize}";
        Accord.Math.Vector3 T = new Accord.Math.Vector3(40, 30, 10);
        // Accord.Math.Matrix3x3 R = Utility.CreateRotationMatrix(0, 0, 0);
        Accord.Math.Matrix3x3 R = Utility.CreateRotationMatrix(0, 90, 0);
 
        // Accord.Math.Matrix3x3 S = Utility.ConstructAccordMatrix3x3(2);
        Accord.Math.Matrix3x3 S = Utility.ConstructAccordMatrix3x3(
            1, 0, 0, 
            0, 1, 0, 
            0, 0, 1);

        CreateExportPointCloud(pcloudSize, false, pcloudName, R, T, S);
    }

    private void CreatePC5() 
    {
        int pcloudSize = 5;
        string pcloudName = $"pc{pcloudSize}";
        Accord.Math.Vector3 T = new Accord.Math.Vector3(20, 40, 10);
        // Accord.Math.Matrix3x3 R = Utility.CreateRotationMatrix(0, 0, 0);
        Accord.Math.Matrix3x3 R = Utility.CreateRotationMatrix(0, 0, 0);
 
        // Accord.Math.Matrix3x3 S = Utility.ConstructAccordMatrix3x3(2);
        Accord.Math.Matrix3x3 S = Utility.ConstructAccordMatrix3x3(
            2, 0, 0, 
            0, 3, 0, 
            0, 0, 1);

        CreateExportPointCloud(pcloudSize, false, pcloudName, R, T, S);
    }

}
