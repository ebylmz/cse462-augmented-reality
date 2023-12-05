using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class ImportPointCloud : MonoBehaviour
{
    [SerializeField] private string fileName;
    [SerializeField] private Material material;

    private void Start()
    {
        Import();
    }

    public void Import()
    {
        string path = Path.Combine(Application.dataPath, "Resources", $"{fileName}.txt");

        // Read all lines from the text file into an array
        string[] lines = File.ReadAllLines(path);

        if (lines.Length < 1) {
            Debug.LogError("File is empty or does not contain point cloud data.");
            return;
        }

        // Read the number of points from the first line
        if (!int.TryParse(lines[0], out int numberOfPoints)) {
            Debug.LogError("Invalid number of points specified in the file.");
            return;
        }

        Vector3 sumPositions = Vector3.zero; // Initialize a vector to store sum of positions

        // Loop through each line of data in the file (skipping the first line)
        for (int i = 1; i <= numberOfPoints; ++i) {
            if (i >= lines.Length) {
                Debug.LogError("Insufficient data for the specified number of points.");
                break;
            }

            string[] line = lines[i].Split(' ');

            // Check if the line contains at least 3 elements and parse them to floats (X, Y, Z)
            if (line.Length >= 3 &&
                float.TryParse(line[0], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float x) &&
                float.TryParse(line[1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float y) &&
                float.TryParse(line[2], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float z))
            {
                sumPositions += new Vector3(x, y, z); // Add the position to the sum

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.transform.position = new Vector3(x, y, z);
                sphere.GetComponent<Renderer>().material = material;
                sphere.name = $"P{i:D3}"; // Set the name of the sphere using the loop index
                sphere.transform.parent = transform;
            }
            else {
                Debug.LogError($"Invalid data at line {i + 1}. Skipping...");
            }
        }

        // Calculate centroid of the point cloud (average position)
        Vector3 centroid = sumPositions / Mathf.Max(1, numberOfPoints); // Avoid division by zero
        Debug.Log($"Centroid of the point cloud: {centroid}");
    }

}
