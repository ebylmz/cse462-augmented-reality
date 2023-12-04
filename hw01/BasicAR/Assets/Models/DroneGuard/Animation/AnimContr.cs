using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AnimContr : MonoBehaviour
{
    public Animator anim;

    void Start()
    {
        anim = GetComponent<Animator>();
    }

    void OnMouseDown()
    {
        anim.Play("ShutDown");
    }

    void OnMouseUp()
    {
        anim.Play("WakeUp");
    }

    void OnMouseDrag()
    {
        anim.Play("Destroyed");
    }
}