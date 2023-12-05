using System.Collections.Generic;
using System;
using UnityEngine;
using Accord.Math;
using Accord.Math.Optimization;
using Accord.Statistics.Models.Regression.Fitting;

public class OptimalTransformation : MonoBehaviour
{
    [SerializeField] private Material materialTransformedPoints;
    [SerializeField] private GameObject pcloud1;
    [SerializeField] private GameObject pcloud2;

    [SerializeField] private float alignmentThreshold = 1f; 
    [SerializeField] private int maxIterationNormal = 10000;
    [SerializeField] private int maxIterationScaled = 1000;

    public bool displayTransformationLines = false;     // Visualization option
    public bool enableScale = false;                    // Registeration option

    private List<GameObject> transformedPoints = new List<GameObject>();
    private List<LineRenderer> transformationLines = new List<LineRenderer>();

    private List<Accord.Math.Vector3> points1;
    private List<Accord.Math.Vector3> points2;

    void Start()
    {
        // Initialize lists to store points
        points1 = ExtractPoints(pcloud1);
        points2 = ExtractPoints(pcloud2);
    }

    public void AlignPointClouds(out Accord.Math.Vector3 translation, out Accord.Math.Matrix3x3 rotation, out Accord.Math.Matrix3x3 scale)
    {
        // Clear previos aligment outputs (if any)
        Clear();

        // Leverage RANSAC algorithm to find/estimate the transformation parameters
        int numInliners = GetTransformationRANSAC(points1, points2, out translation, out rotation, out scale);

        // TODO: Check the number of inliers
        CreateTransformedPoints(translation, rotation, scale);
        if (displayTransformationLines)
            CreateTransformationLines();

        Debug.Log("Alignment completed.");
    }

    // Extracts points from a given point cloud GameObject
    private List<Accord.Math.Vector3> ExtractPoints(GameObject pointCloud)
    {
        List<Accord.Math.Vector3> extractedPoints = new List<Accord.Math.Vector3>();

        for (int i = 0; i < pointCloud.transform.childCount; ++i) {
            UnityEngine.Vector3 position = pointCloud.transform.GetChild(i).position;
            Accord.Math.Vector3 point = new Accord.Math.Vector3(position.x, position.y, position.z);
            extractedPoints.Add(point);
        }

        return extractedPoints;
    }

    public void Clear()
    {
        ClearTransformedPoints();
        ClearTransformationLines();
        pcloud2.SetActive(true);
    }

    private void CreateTransformedPoints(Accord.Math.Vector3 T, Accord.Math.Matrix3x3 R, Accord.Math.Matrix3x3 S)
    {
        for (int i = 0; i < points1.Count; ++i) {
            Accord.Math.Vector3 q = ApplyTransformation(points1[i], R, S, T);
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = new UnityEngine.Vector3(q.X, q.Y, q.Z);
            sphere.GetComponent<Renderer>().material = materialTransformedPoints;
            sphere.name = $"P{i:D3}"; 
            sphere.transform.parent = transform;
            transformedPoints.Add(sphere);
        }
    }

    private Accord.Math.Vector3 ApplyTransformation(Accord.Math.Vector3 P, Accord.Math.Matrix3x3 R, Accord.Math.Matrix3x3 S, Accord.Math.Vector3 T)
    {
        return enableScale ? (S * R * P + T) : (R * P + T);
    }

    private void ClearTransformedPoints()
    {
        foreach (GameObject point in transformedPoints)
            Destroy(point);
        transformedPoints.Clear();
    }

    public void DisplayTransformationLines(bool display)
    {
        if (display) {
            if (!displayTransformationLines) {
                CreateTransformationLines();
                pcloud2.SetActive(false);
            }
        }
        else {
            pcloud2.SetActive(true);
            ClearTransformationLines();
        }
        displayTransformationLines = display;
    }

    private void CreateTransformationLines()
    {
        LineRenderer lineRenderer;
        
        for (int i = 0; i < transformedPoints.Count; ++i) {
            Transform p = pcloud1.transform.GetChild(i);
            UnityEngine.Vector3 q = transformedPoints[i].transform.position;

            // Check if the GameObject already has a LineRenderer component
            lineRenderer = p.gameObject.GetComponent<LineRenderer>();

            // If there's no LineRenderer component, add a new one
            if (lineRenderer == null) {
                lineRenderer = p.gameObject.AddComponent<LineRenderer>();
                lineRenderer.startColor = Color.blue;
                lineRenderer.endColor = Color.red;
                lineRenderer.startWidth = 0.2f;
                lineRenderer.endWidth = 0.2f;
            }

            lineRenderer.SetPosition(0, p.position);
            lineRenderer.SetPosition(1, q);

            transformationLines.Add(lineRenderer);
        } 
    }

    // Function to hide or remove transformation lines based on the view option status
    private void ClearTransformationLines()
    {
        foreach (LineRenderer lineRenderer in transformationLines)
            Destroy(lineRenderer);
        transformationLines.Clear();
    }

    // Calculate the best transformation between two point clouds
    private int GetTransformationRANSAC(List<Accord.Math.Vector3> points1, List<Accord.Math.Vector3> points2, out Accord.Math.Vector3 T, out Accord.Math.Matrix3x3 R, out Accord.Math.Matrix3x3 S) 
    {
        List<Accord.Math.Vector3> points1Transformed = new List<Accord.Math.Vector3>();

        for (int i = 0; i < points2.Count; ++i)
            points1Transformed.Add(new Accord.Math.Vector3());

        // Initialize tranformation matrices with default values (R and S are initially identity matrices)
        T = new Accord.Math.Vector3(0f, 0f, 0f);
        R = Utility.ConstructAccordMatrix3x3(1); 
        S = Utility.ConstructAccordMatrix3x3(1);

        int numBestInliers = 0;
        double minError= Double.MaxValue;
        int maxIteration = enableScale ? maxIterationScaled : maxIterationNormal;

        Accord.Math.Vector3 translation; 
        Accord.Math.Matrix3x3 rotation;
        Accord.Math.Matrix3x3 scale = Utility.ConstructAccordMatrix3x3(1);

        int iteration;
        for (iteration = 0; iteration < maxIteration; ++iteration) {
            // Random sampling
            List<Accord.Math.Vector3> sample1 = GetRandomPointsFromPointCloud(points1, 3);
            List<Accord.Math.Vector3> sample2 = GetRandomPointsFromPointCloud(points2, 3);

            if (enableScale) {
                scale = GetScaleNumerically(sample1, sample2);

                if (Matrix.IsSingular(MatrixToArray(scale)))
                    continue;

                Accord.Math.Matrix3x3 inverseScale = scale.Inverse();

                // Apply the inverse scaling
                for (int j = 0; j < sample2.Count; ++j)
                    sample2[j] =  inverseScale * sample2[j];
            }
            
            GetTransformation(sample1, sample2, out translation, out rotation);

            if (enableScale) {
                translation = scale * translation;
            }

            // Apply transformation and count the overlapping points
            for (int j = 0; j < points1.Count; ++j)
                points1Transformed[j] = ApplyTransformation(points1[j], rotation, scale, translation);

            double error;
            int numInliers = CountOverlappingPoints(points1Transformed, points2, out error);

            // Update the best transformation
            if (numInliers > numBestInliers || error < minError) {
                numBestInliers = numInliers;
                minError = error;
                T = translation;
                R = rotation;
                S = scale;
                // Stop RANSAC
                if (numBestInliers >= points1.Count / 2)
                    break;
            }
        }
        Debug.Log($"Best inliers: {numBestInliers}, Total Iterations: {iteration}, Error: {minError:F2}");

        return numBestInliers;
    }

    private double[,] MatrixToArray(Accord.Math.Matrix3x3 m)
    {
        double[,] arr = new double[3, 3];

        arr[0, 0] = m.V00;
        arr[0, 1] = m.V01;
        arr[0, 2] = m.V02;

        arr[1, 0] = m.V10;
        arr[1, 1] = m.V11;
        arr[1, 2] = m.V12;

        arr[2, 0] = m.V20;
        arr[2, 1] = m.V21;
        arr[2, 2] = m.V22;

        return arr;
    }

    private List<float> GetDistances(List<Accord.Math.Vector3> s, int n)
    {
        List<float> distances = new List<float>((n - 1) * (n - 2) / 2); 
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j)
                distances.Add(Utility.DistanceTwoAccordVector(s[i], s[j]));
        return distances;
    }

    private bool CheckCorrespondence(List<Accord.Math.Vector3> s1, List<Accord.Math.Vector3> s2, int n, float noise)
    {
        for (int i = 0; i < n; ++i) { 
            for (int j = i + 1; j < n; ++j) {
                float d1 = Utility.DistanceTwoAccordVector(s1[i], s1[j]);
                float d2 = Utility.DistanceTwoAccordVector(s2[i], s2[j]);
                if (Mathf.Abs(d1 - d2) > noise)
                    return false;
            }
        }
        return true;
    }

    // Function to get n random points from a point cloud
    private List<Accord.Math.Vector3> GetRandomPointsFromPointCloud(List<Accord.Math.Vector3> pointCloud, int n) 
    {
        List<int> randomIndices = Utility.GetDistinctRandomNumbers(n, pointCloud.Count);
        List<Accord.Math.Vector3> randomPoints = new List<Accord.Math.Vector3>();
        foreach (int index in randomIndices) 
            randomPoints.Add(pointCloud[index]);
        return randomPoints;
    }

    private void GetTransformation(List<Accord.Math.Vector3> p3, List<Accord.Math.Vector3> q3, out Accord.Math.Vector3 translation, out Accord.Math.Matrix3x3 rotation)
    {
        Accord.Math.Vector3 centroid1 = GetCentroid(p3);
        Accord.Math.Vector3 centroid2 = GetCentroid(q3);

        Accord.Math.Matrix3x3 matrix1 = Normalize3PointMatrix(p3, centroid1);
        Accord.Math.Matrix3x3 matrix2 = Normalize3PointMatrix(q3, centroid2);

        Accord.Math.Matrix3x3 H = matrix1 * matrix2.Transpose();

        H.SVD(out Accord.Math.Matrix3x3 U, out Accord.Math.Vector3 S, out Accord.Math.Matrix3x3 V);

        rotation = V * U.Transpose();

        if (rotation.Determinant < 0) {
            rotation.SVD(out Accord.Math.Matrix3x3 U_temp, out Accord.Math.Vector3 S_temp, out Accord.Math.Matrix3x3 V_temp);
            V_temp.V20 = -V_temp.V00;
            V_temp.V21 = -V_temp.V01;
            V_temp.V22 = -V_temp.V02;
            rotation = V_temp * U_temp.Transpose();
        }

        translation = centroid2 - rotation * centroid1;
    }

    private void GetDelta(Accord.Math.Vector3 p1, Accord.Math.Vector3 p2, out float deltaX, out float deltaY, out float deltaZ)
    {
        deltaX = p1.X - p2.X;
        deltaY = p1.Y - p2.Y;
        deltaZ = p1.Z - p2.Z;
    }

    private Accord.Math.Matrix3x3 GetScaleNumerically(List<Accord.Math.Vector3> s1, List<Accord.Math.Vector3> s2)
    {
        int n = 3; // point pairs
        
        double[][] inputs = new double[n][];
        double[] outputs = new double[n];

        int pair = 0;
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                float dx, dy, dz, dxPrime, dyPrime, dzPrime;
                GetDelta(s2[i], s2[j], out dx, out dy, out dz); 
                GetDelta(s1[i], s1[j], out dxPrime, out dyPrime, out dzPrime); 

                outputs[pair] = System.Math.Pow(dx, 2) + System.Math.Pow(dy, 2) + System.Math.Pow(dz, 2); 
                inputs[pair] = new double[3]; // Each inner array has 3 elements
                inputs[pair][0] = dxPrime;
                inputs[pair][1] = dyPrime;
                inputs[pair][2] = dzPrime;
                ++pair;
            }
        }

        LeastSquaresFunction function = (double[] parameters, double[] input) =>
        {
            return System.Math.Pow(parameters[0] * input[0], 2) +
                System.Math.Pow(parameters[1] * input[1], 2) +
                System.Math.Pow(parameters[2] * input[2], 2); 
        };

        LeastSquaresGradientFunction gradient = (double[] parameters, double[] input, double[] result) =>
        {
            result[0] = 2 * input[0] * parameters[0];
            result[1] = 2 * input[1] * parameters[1];
            result[2] = 2 * input[2] * parameters[2];
        };

        // Create a new Levenberg-Marquardt algorithm
        var gn = new LevenbergMarquardt(parameters: 3)
        {
            Function = function,
            Gradient = gradient,
            Solution = new[] {1.0, 1.0, 1.0} 
        };

        // Find the minimum value
        gn.Minimize(inputs, outputs);

        // Extract the scaling parameter
        float sx = (float) gn.Solution[0];
        float sy = (float) gn.Solution[1];
        float sz = (float) gn.Solution[2];

        Debug.Log($"Found Scaling Factors: sx = {sx:F2}, sy = {sy:F2}, sz = {sz:F2} (Numeric)");

        return Utility.ConstructAccordMatrix3x3(
            sx, 0f, 0f,
            0f, sy, 0f,
            0f, 0f, sz
        );
    }
    
    private Accord.Math.Matrix3x3 GetScaleAlgebraically(List<Accord.Math.Vector3> s1, List<Accord.Math.Vector3> s2)
    {
        // Ax = b
        // x = A'b
        Accord.Math.Vector3 b = new Accord.Math.Vector3();

        b.X = Mathf.Pow(s1[0].X - s1[1].X, 2) + Mathf.Pow(s1[0].Y - s1[1].Y, 2) + Mathf.Pow(s1[0].Z - s1[1].Z, 2);
        b.Y = Mathf.Pow(s1[0].X - s1[2].X, 2) + Mathf.Pow(s1[0].Y - s1[2].Y, 2) + Mathf.Pow(s1[0].Z - s1[2].Z, 2);
        b.Z = Mathf.Pow(s1[1].X - s1[2].X, 2) + Mathf.Pow(s1[1].Y - s1[2].Y, 2) + Mathf.Pow(s1[1].Z - s1[2].Z, 2);

        Accord.Math.Matrix3x3 A = new Accord.Math.Matrix3x3();

        A.V00 = Mathf.Pow(s2[0].X - s2[1].X, 2);
        A.V10 = Mathf.Pow(s2[0].X - s2[2].X, 2);
        A.V20 = Mathf.Pow(s2[1].X - s2[2].X, 2);

        A.V01 = Mathf.Pow(s2[0].Y - s2[1].Y, 2);
        A.V11 = Mathf.Pow(s2[0].Y - s2[2].Y, 2);
        A.V21 = Mathf.Pow(s2[1].Y - s2[2].Y, 2);

        A.V02 = Mathf.Pow(s2[0].Z - s2[1].Z, 2);
        A.V12 = Mathf.Pow(s2[0].Z - s2[2].Z, 2);
        A.V22 = Mathf.Pow(s2[1].Z - s2[2].Z, 2);

        Accord.Math.Matrix3x3 S = Utility.ConstructAccordMatrix3x3(1);
        if (!Matrix.IsSingular(MatrixToArray(A))) {
            // A is non-reversable
            Accord.Math.Vector3 x = A.Inverse() * b;            
            S.V00 = Mathf.Sqrt(x.X);        
            S.V11 = Mathf.Sqrt(x.Y);        
            S.V22 = Mathf.Sqrt(x.Z);        
        }
        
        Debug.Log($"Found Scaling Factors: sx = {S.V00:F2}, sy = {S.V11:F2}, sz = {S.V22:F2} (Algebra)");

        return S; 
    }

    private Accord.Math.Matrix3x3 Normalize3PointMatrix(List<Accord.Math.Vector3> points, Accord.Math.Vector3 centroid)
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

    private Accord.Math.Vector3 GetCentroid(List<Accord.Math.Vector3> points)
    {
        Accord.Math.Vector3 centroid = new Accord.Math.Vector3(0f, 0f, 0f);
    
        foreach (Accord.Math.Vector3 point in points)
            centroid += point;

        centroid /= points.Count;
        return centroid;
    }

    private int CountOverlappingPoints(List<Accord.Math.Vector3> initials, List<Accord.Math.Vector3> targets, out double totalError)
    {
        totalError = 0.0;
        int count = 0;

        for (int i = 0; i < initials.Count; ++i) {
            double minError = Double.MaxValue; 
            for (int j = 0; j < targets.Count; ++j) {
                double error = Utility.SquaredDistanceTwoAccordVector(initials[i], targets[j]);
                if (error <= minError) 
                    minError = error;
            }
            totalError += minError;
            if (minError < alignmentThreshold)
                ++count;
        }

        return count;
    }   
}