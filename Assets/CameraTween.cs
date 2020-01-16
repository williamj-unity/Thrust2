using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraTween : MonoBehaviour
{
    public Transform ship;

    Vector3 acceleration;
    Vector3 prevPosition;
    Vector3 targetPosition;
    public float accelerationCoef;
    public float momentumDamping;

    // Start is called before the first frame update
    void Start()
    {
        prevPosition = transform.position;
        acceleration = Vector3.zero;
        targetPosition = ship.position;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        targetPosition = ship.position;
        targetPosition.z = transform.position.z;
        UpdatePosition();
    }

    void UpdatePosition()
    {
        acceleration = acceleration + (targetPosition - transform.position);

        float timeStep = Time.deltaTime;
        float timeStepSq = timeStep * timeStep;

        Vector3 velocity = (transform.position - prevPosition) * momentumDamping;
        Vector3 next = transform.position + (velocity) + accelerationCoef * acceleration * timeStepSq;
        prevPosition = transform.position;
        transform.position = next;
        acceleration = Vector3.zero;
    }
}
