using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InteractableBehavior : MonoBehaviour
{
    private bool firstCollision = true;

    private void OnTriggerEnter(Collider collider)
    {
        if(firstCollision)
        {
            var rb = this.GetComponent<Rigidbody>();
            rb.useGravity = true;
            firstCollision = false;
        }
    }

    private void OnEnable()
    {
        var rb = this.GetComponent<Rigidbody>();
        rb.useGravity = false;
        firstCollision = true;
    }
}
