using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Accord.Math;

public class Utility : MonoBehaviour
{
    public static Accord.Math.Matrix3x3 ConstructAccordMatrix3x3(params float[] values)
    {
        Accord.Math.Matrix3x3 matrix = new Accord.Math.Matrix3x3();

        if (values.Length == 1) {
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
        else if (values.Length != 9) {
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

    public static Matrix3x3 CreateRotationMatrix(float degreeX, float degreeY, float degreeZ)
    {
        float x = AngleToRadians(degreeX);
        float y = AngleToRadians(degreeY);
        float z = AngleToRadians(degreeZ);

        Matrix3x3 Rx = ConstructAccordMatrix3x3(
            1, 0, 0,
            0, (float)Mathf.Cos(x), -(float)Mathf.Sin(x),
            0, (float)Mathf.Sin(x), (float)Mathf.Cos(x));

        Matrix3x3 Ry = ConstructAccordMatrix3x3(
            (float)Mathf.Cos(y), 0, (float)Mathf.Sin(y),
            0, 1, 0,
            -(float)Mathf.Sin(y), 0, (float)Mathf.Cos(y));

        Matrix3x3 Rz = ConstructAccordMatrix3x3(
            (float)Mathf.Cos(z), -(float)Mathf.Sin(z), 0,
            (float)Mathf.Sin(z), (float)Mathf.Cos(z), 0,
            0, 0, 1);

        return Rz * Ry * Rx;
    }

    // Function to get n distinct random numbers within a range
    public static List<int> GetDistinctRandomNumbers(int n, int range) 
    {
        List<int> randomNumbers = new List<int>();
        HashSet<int> numberSet = new HashSet<int>();
        
        // Check if n is greater than the range
        if (n >= range) {
            for (int i = 0; i < range; ++i) 
                randomNumbers.Add(i);
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

    public static double SquaredDistanceTwoAccordVector(Accord.Math.Vector3 p1, Accord.Math.Vector3 p2)
    {
        return Mathf.Pow(p1.X - p2.X, 2) + Mathf.Pow(p1.Y - p2.Y, 2) + Mathf.Pow(p1.Z - p2.Z, 2);
    }

    public static float DistanceTwoAccordVector(Accord.Math.Vector3 p1, Accord.Math.Vector3 p2)
    {
        return (float)Mathf.Sqrt(Mathf.Pow(p1.X - p2.X, 2) + Mathf.Pow(p1.Y - p2.Y, 2) + Mathf.Pow(p1.Z - p2.Z, 2));
    }

    public static float AngleToRadians(float angleInDegrees)
    {
        return angleInDegrees * Mathf.PI / 180;
    }

    public static string Vector3ToString(Accord.Math.Vector3 vector)
    {
        return $"[{vector.X:F2}, {vector.Y:F2}, {vector.Z:F2}]";
    }

    public static string Matrix3x3ToString(Matrix3x3 matrix)
    {
        return $"|{matrix.V00:F2}, {matrix.V01:F2}, {matrix.V02:F2}|\n" +
            $"|{matrix.V10:F2}, {matrix.V11:F2}, {matrix.V12:F2}|\n" +
            $"|{matrix.V20:F2}, {matrix.V21:F2}, {matrix.V22:F2}|";
    }

    public static string DisplayTransformationText(Accord.Math.Vector3 translation, Accord.Math.Matrix3x3 rotation, Accord.Math.Matrix3x3 scale, bool enableScale)
    {
        string text = $"Translation:\n {Vector3ToString(translation)}\n" + 
            $"Rotation:\n{Matrix3x3ToString(rotation)}";
        if (enableScale)
            text += $"\nScale:\n{Matrix3x3ToString(scale)}";

        return text;
    }
}
