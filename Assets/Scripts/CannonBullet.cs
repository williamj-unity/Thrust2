using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CannonBullet : MonoBehaviour
{
    public float speed = 10f;
    Vector3 startVelocity;
    // Start is called before the first frame update
    void Start()
    {
        Rigidbody2D rb = GetComponent<Rigidbody2D>();
        rb.drag = 0;
        rb.mass = 0.1f;
        rb.gravityScale = 0;

        rb.velocity = startVelocity;
        rb.AddForce(transform.forward * speed);
        Destroy(gameObject, 5.0f);
    }

    internal void SetVeloctiy(Vector3 velocity)
    {
        startVelocity = velocity;
    }
}
