using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using Accord.Math;

public class TransformationManager : MonoBehaviour
{

    [SerializeField] private OptimalTransformation transformer;
    [SerializeField] private TMP_Text toggleDisplayLinesButtonText; 
    [SerializeField] private TMP_Text toggleScaleButtonText; 
    [SerializeField] private TMP_Text transformationText;
    
    public void Run()
    {
        transformer.AlignPointClouds(out Accord.Math.Vector3 T, out Accord.Math.Matrix3x3 R, out Accord.Math.Matrix3x3 S);
        transformationText.text = Utility.DisplayTransformationText(T, R, S, transformer.enableScale);
    }

    public void Clear()
    {
        transformer.Clear();
        transformationText.text = "Translation\nRotation\nScale";
    }

    public void ToggleScale()
    {
        transformer.enableScale = !transformer.enableScale; 
        
        toggleScaleButtonText.color = transformer.enableScale ? Color.green : Color.red;
    }

    public void ToggleDisplayLines()
    {
        transformer.DisplayTransformationLines(!transformer.displayTransformationLines);
        toggleDisplayLinesButtonText.color = transformer.displayTransformationLines ? Color.green : Color.red;
    }
}
