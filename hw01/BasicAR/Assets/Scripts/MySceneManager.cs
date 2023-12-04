using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class MySceneManager : MonoBehaviour
{
    // Load the next scene in the build settings
    public void LoadNextScene()
    {
        int currentSceneIndex = SceneManager.GetActiveScene().buildIndex;
        Debug.Log($"Current scene: {currentSceneIndex}");
        int nextSceneIndex = (currentSceneIndex + 1) % SceneManager.sceneCountInBuildSettings;
        SceneManager.LoadScene(nextSceneIndex);
    }

    // Load the previous scene in the build settings
    public void LoadPreviousScene()
    {
        int currentSceneIndex = SceneManager.GetActiveScene().buildIndex;
        int previousSceneIndex = currentSceneIndex - 1;
        if (previousSceneIndex < 0)
        {
            previousSceneIndex = SceneManager.sceneCountInBuildSettings - 1;
        }
        SceneManager.LoadScene(previousSceneIndex);
    }
}