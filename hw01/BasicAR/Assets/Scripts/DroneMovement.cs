using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DroneMovement : MonoBehaviour
{
    public float movementSpeed = 20.0f; // Adjust the speed as needed
    private bool moveForward = false;
    private bool moveBackward = false;
    private bool moveLeft = false;
    private bool moveRight = false;
    private bool moveUp = false;
    private bool moveDown = false;

    void Update()
    {
        // Check button states and move the object accordingly
        Vector3 movement = Vector3.zero;

        if (moveForward)
            movement += transform.forward;
        if (moveBackward)
            movement -= transform.forward;
        if (moveLeft)
            movement -= transform.right;
        if (moveRight)
            movement += transform.right;
        if (moveUp)
            movement += transform.up;
        if (moveDown)
            movement -= transform.up;

        transform.Translate(movement.normalized * Time.deltaTime * movementSpeed);
    }

    public void OnForwardButtonDown()
    {
        moveForward = true;
    }

    public void OnForwardButtonUp()
    {
        moveForward = false;
    }

    public void OnBackwardButtonDown()
    {
        moveBackward = true;
    }

    public void OnBackwardButtonUp()
    {
        moveBackward = false;
    }

    public void OnLeftButtonDown()
    {
        moveLeft = true;
    }

    public void OnLeftButtonUp()
    {
        moveLeft = false;
    }

    public void OnRightButtonDown()
    {
        moveRight = true;
    }

    public void OnRightButtonUp()
    {
        moveRight = false;
    }

    public void OnUpButtonDown()
    {
        moveUp = true;
    }

    public void OnUpButtonUp()
    {
        moveUp = false;
    }

    public void OnDownButtonDown()
    {
        moveDown = true;
    }

    public void OnDownButtonUp()
    {
        moveDown = false;
    }
}