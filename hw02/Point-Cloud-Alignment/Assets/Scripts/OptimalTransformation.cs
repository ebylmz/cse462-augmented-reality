using System.Collections.Generic;
using UnityEngine;
using Accord.Math;
using UnityEngine.UI;
using TMPro;

// using Accord.Math.Decompositions;
// using Accord.Imaging;

public class OptimalTransformation : MonoBehaviour
{
    [SerializeField] private Material materialTransformedPoints;
    [SerializeField] private TMP_Text toggleVisualizationButtonText; 
    [SerializeField] private TMP_Text toggleScaleButtonText; 

    [SerializeField] private TMP_Text toggleRunButtonText; 
    [SerializeField] private TMP_Text transformationText;
    // [SerializeField] private TMP_Text scaleText;

    // Public variables accessible in the Unity Inspector
    public GameObject pointCloud_1;
    public GameObject pointCloud_2;
    public float threshold = 0.3f; // make it constant
    public bool transformPointCloud = true;
    public bool drawTransformLine = true;
    public int iteration = 10000;
    public bool run = false;

    public bool viewLines; // Visualization option
    public bool enableScale; // Registeration option

    // Private variables to track alignment status and point lists
    private bool isAligned = false;
    private List<Accord.Math.Vector3> points_1;
    private List<Accord.Math.Vector3> points_2;

    void Start()
    {
        // Initialize lists to store points
        points_1 = ExtractPoints(pointCloud_1);
        points_2 = ExtractPoints(pointCloud_2);
    }

    void Update()
    {
        // Perform alignment if conditions are met
        if (!isAligned && run)
        {
            // TODO: two registering option
            // TODO: two display option
            AlignPointClouds();
            // DisplayScaleText(scale);
        }
    }

    void DisplayTransformationText(Accord.Math.Vector3 translation, Accord.Math.Matrix3x3 rotation, Accord.Math.Matrix3x3 scale)
    {
        string text = $"Translation: {Vector3ToString(translation)}\n" +
                    $"Rotation:\n{Matrix3x3ToString(rotation)}\n" +
                    $"Scale:\n{Matrix3x3ToString(scale)}";

        // check if scaling is used

        transformationText.text = text;
    }

    // Function to convert a Vector3 to string
    string Vector3ToString(Accord.Math.Vector3 vector)
    {
        return $"({vector.X:F2}, {vector.Y:F2}, {vector.Z:F2})";
    }

    // Function to convert a Matrix3x3 to string
    string Matrix3x3ToString(Matrix3x3 matrix)
    {
        return $"{matrix.V00:F2}, {matrix.V01:F2}, {matrix.V02:F2}\n" +
            $"{matrix.V10:F2}, {matrix.V11:F2}, {matrix.V12:F2}\n" +
            $"{matrix.V20:F2}, {matrix.V21:F2}, {matrix.V22:F2}";
    }

    public void ToggleRun()
    {
        run = !run;
        toggleRunButtonText.text = run ? "Reset" : "Run";
    }

    // Method to toggle the visualization
    public void ToggleVisualization()
    {
        viewLines = !viewLines;
        // Change the button text based on the boolean value
        // toggleVisualizationButtonText.text = viewLines ? "Remove Line" : "View Line";
        toggleVisualizationButtonText.color = viewLines ? Color.green : Color.red;
    }

    // Method to toggle the algorithm
    public void ToggleTransformationAlgorithm()
    {
        enableScale = !enableScale;
        // Change the button text based on the boolean value
        toggleScaleButtonText.color = enableScale ? Color.green : Color.red;
    }

    // Extracts points from a given point cloud GameObject
    List<Accord.Math.Vector3> ExtractPoints(GameObject pointCloud)
    {
        List<Accord.Math.Vector3> extractedPoints = new List<Accord.Math.Vector3>();

        for (int i = 0; i < pointCloud.transform.childCount; i++)
        {
            UnityEngine.Vector3 position = pointCloud.transform.GetChild(i).position;
            Accord.Math.Vector3 point = new Accord.Math.Vector3(position.x, position.y, position.z);
            extractedPoints.Add(point);
        }

        return extractedPoints;
    }

    // Aligns the point clouds using ICP and RANSAC algorithms
    void AlignPointClouds()
    {
        List<Accord.Math.Vector3> points_2_tranformed;
        // Get the best transformation parameters
        Accord.Math.Vector3 translation;
        Accord.Math.Matrix3x3 rotation;
        Accord.Math.Matrix3x3 scale;

        GetBestTransformation(points_1, points_2, out translation, out rotation, out scale, out points_2_tranformed);

        // 2 different visualization


        if (viewLines) // Show the transformed points (second sets) with its movement as a line.
            CreateTransformationLines(points_2_tranformed);
        else // Show the original and aligned points (with three different colors)       
            CreateTransformedPoints(points_2_tranformed);

        DisplayTransformationText(translation, rotation, scale);

        isAligned = true; // is this necessary
        Debug.Log("Alignment completed.");
    }

    void CreateTransformedPoints(List<Accord.Math.Vector3> transformedPoints)
    {
        for (int i = 0; i < transformedPoints.Count; ++i) {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = new UnityEngine.Vector3(transformedPoints[i].X, transformedPoints[i].Y, transformedPoints[i].Z);
            sphere.GetComponent<Renderer>().material = materialTransformedPoints;
            sphere.name = $"P {i}"; 
            sphere.transform.parent = transform;
        }
    }

    void CreateTransformationLines(List<Accord.Math.Vector3> transformedPoints)
    {
        for (int i = 0; i < pointCloud_2.transform.childCount; ++i)
            VisualizeTransformationLine(pointCloud_2.transform.GetChild(i), transformedPoints[i]);
    }

    // Visualize transformation lines between original and transformed points
    void VisualizeTransformationLine(Transform originalPoint, Accord.Math.Vector3 transformedPoint)
    {
        LineRenderer lineRenderer = originalPoint.gameObject.AddComponent<LineRenderer>();
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = Color.red;
        lineRenderer.endColor = Color.green;
        lineRenderer.startWidth = 0.2f;
        lineRenderer.endWidth = 0.2f;
        lineRenderer.SetPosition(0, originalPoint.position);
        lineRenderer.SetPosition(1, new UnityEngine.Vector3(transformedPoint.X, transformedPoint.Y, transformedPoint.Z));
    }

    Accord.Math.Vector3 Unity2AccordVector(UnityEngine.Vector3 vector)
    {
        return new Accord.Math.Vector3(vector.x, vector.y, vector.z); 
    }

    // Calculate the best transformation between two point clouds
    void GetBestTransformation(List<Accord.Math.Vector3> points_1, List<Accord.Math.Vector3> points_2, out Accord.Math.Vector3 T, out Accord.Math.Matrix3x3 R, out Accord.Math.Matrix3x3 S, out List<Accord.Math.Vector3> points_2_transformed){
        
        int numBestInliers = 0;
        points_2_transformed = new List<Accord.Math.Vector3>();

        for (int i = 0; i < points_2.Count; ++i)
            points_2_transformed.Add(new Accord.Math.Vector3());

        // Initialize tranformation matrices with default values
        T = new Accord.Math.Vector3(0f, 0f, 0f);
        R = ConstructAccordMatrix3x3(1); // Create identity matrix
        S = ConstructAccordMatrix3x3(1);

        // TODO: create RANSAC function and apply this procedure inside
        for (int i = 0; i < iteration && (numBestInliers < points_1.Count / 2); ++i) {
            Debug.Log($"Iteration {i + 1}");

            List<Accord.Math.Vector3> selectedPoints_1 = GetRandomPointsFromPointCloud(points_1, 3);
            List<Accord.Math.Vector3> selectedPoints_2 = GetRandomPointsFromPointCloud(points_2, 3);

            Accord.Math.Vector3 translation;
            Accord.Math.Matrix3x3 rotation;
            Accord.Math.Matrix3x3 scale;

            GetTransformation(selectedPoints_1, selectedPoints_2, 
                out translation, out rotation, out scale);

            // Apply transformation
            for (int j = 0; j < points_2.Count; ++j)
                points_2_transformed[j] = rotation * points_2[j] + translation;

            int numInliers = CountOverlappingPoints(points_1, points_2_transformed, threshold);

            // Update the best transformation
            if (numInliers > numBestInliers) {
                numBestInliers = numInliers;
                T = translation;
                R = rotation;
                S = scale;
                Debug.Log($"Number of inliers: {numBestInliers}");
            }
        }
        Debug.Log($"Best inliers: {numBestInliers}");
    }

    Accord.Math.Matrix3x3 ConstructAccordMatrix3x3(params float[] values)
    {
        Accord.Math.Matrix3x3 matrix = new Accord.Math.Matrix3x3();

        if (values.Length == 1)
        {
            float v = values[0];
            matrix.V00 = v;
            matrix.V01 = 0;
            matrix.V02 = 0;

            matrix.V10 = 0;
            matrix.V11 = v;
            matrix.V12 = 0;

            matrix.V20 = 0;
            matrix.V21 = 0;
            matrix.V22 = v;
        }
        else if (values.Length != 9)
        {
            Debug.Log("Error. The input list must contain exactly 9 elements for a 3x3 matrix.");
        }
        else {
            matrix.V00 = values[0];
            matrix.V01 = values[1];
            matrix.V02 = values[2];

            matrix.V10 = values[3];
            matrix.V11 = values[4];
            matrix.V12 = values[5];

            matrix.V20 = values[6];
            matrix.V21 = values[7];
            matrix.V22 = values[8];
        }

        return matrix;
    }

    // Function to get n distinct random numbers within a range
    List<int> GetDistinctRandomNumbers(int n, int range) 
    {
        List<int> randomNumbers = new List<int>();
        HashSet<int> numberSet = new HashSet<int>();
        
        // Check if n is greater than the range
        if (n >= range) {
            for (int i = 0; i < range; i++) {
                randomNumbers.Add(i);
            }
            return randomNumbers;
        }
        
        while (randomNumbers.Count < n) {
            int random = Random.Range(0, range);
            if (!numberSet.Contains(random)) {
                numberSet.Add(random);
                randomNumbers.Add(random);
            }
        }
        
        return randomNumbers;
    }

    // Function to get n random points from a point cloud
    List<Accord.Math.Vector3> GetRandomPointsFromPointCloud(List<Accord.Math.Vector3> pointCloud, int n) 
    {
        List<int> randomIndices = GetDistinctRandomNumbers(n, pointCloud.Count);
        List<Accord.Math.Vector3> randomPoints = new List<Accord.Math.Vector3>();
        foreach (int index in randomIndices) {
            randomPoints.Add(pointCloud[index]);
        }
        return randomPoints;
    }

    void GetTransformation(List<Accord.Math.Vector3> points3_1, List<Accord.Math.Vector3> points3_2, out Accord.Math.Vector3 translation, out Accord.Math.Matrix3x3 rotation, out Accord.Math.Matrix3x3 scale)
    {
        Accord.Math.Vector3 centroid_1 = GetCentroid(points3_1);
        Accord.Math.Vector3 centroid_2 = GetCentroid(points3_2);

        Accord.Math.Matrix3x3 matrix_1 = Normalize3PointMatrix(points3_1, centroid_1);
        Accord.Math.Matrix3x3 matrix_2 = Normalize3PointMatrix(points3_2, centroid_2);

        Accord.Math.Matrix3x3 H = matrix_1 * matrix_2.Transpose();

        H.SVD(out Accord.Math.Matrix3x3 U, out Accord.Math.Vector3 S, out Accord.Math.Matrix3x3 V);

        Accord.Math.Matrix3x3 R = V * U.Transpose();

        if (R.Determinant < 0) {
            R.SVD(out Accord.Math.Matrix3x3 U_temp, out Accord.Math.Vector3 S_temp, out Accord.Math.Matrix3x3 V_temp);
            V_temp.V20 = -V_temp.V00;
            V_temp.V21 = -V_temp.V01;
            V_temp.V22 = -V_temp.V02;
            R = V_temp * U_temp.Transpose();
        }

        translation = centroid_2 - R * centroid_1;
        rotation = R;
        scale = ConstructAccordMatrix3x3(1); // TODO implement scaling properly
    }

    Accord.Math.Matrix3x3 Normalize3PointMatrix(List<Accord.Math.Vector3> points, Accord.Math.Vector3 centroid)
    {
        Accord.Math.Matrix3x3 matrix = new Accord.Math.Matrix3x3();
        matrix.V00 = points[0].X - centroid.X;
        matrix.V01 = points[0].Y - centroid.Y;
        matrix.V02 = points[0].Z - centroid.Z;

        matrix.V10 = points[1].X - centroid.X;
        matrix.V11 = points[1].Y - centroid.Y;
        matrix.V12 = points[1].Z - centroid.Z;
        
        matrix.V20 = points[2].X - centroid.X;
        matrix.V21 = points[2].Y - centroid.Y;
        matrix.V22 = points[2].Z - centroid.Z;
    
        return matrix;
    }

    Accord.Math.Vector3 GetCentroid(List<Accord.Math.Vector3> points)
    {
        Accord.Math.Vector3 centroid = new Accord.Math.Vector3(0f, 0f, 0f);
    
        foreach (Accord.Math.Vector3 point in points)
        {
            centroid += point;
        }

        centroid /= points.Count;
        return centroid;
    }

    /* Count overlapping points in two point cloud */
    int CountOverlappingPoints(List<Accord.Math.Vector3> points_1, List<Accord.Math.Vector3> points_2, float threshold)
    {
        int numOverlaps = 0;
        int[] overlappingPoints = new int[points_1.Count]; // instead use map

        for (int i = 0; i < points_1.Count; i++)
        {
            for (int j = 0; j < points_2.Count; j++)
            {
                if (DistanceTwoAccordVector(points_1[i], points_2[j]) < threshold && overlappingPoints[i] == 0)
                {
                    numOverlaps++;
                    overlappingPoints[i] = 1;
                    break;
                }
            }
        }

        return numOverlaps;
    }

    public float DistanceTwoAccordVector(Accord.Math.Vector3 point_1, Accord.Math.Vector3 point_2){
        return (float)Mathf.Sqrt(Mathf.Pow(point_1.X - point_2.X, 2) + Mathf.Pow(point_1.Y - point_2.Y, 2) + Mathf.Pow(point_1.Z - point_2.Z, 2));
    }
}