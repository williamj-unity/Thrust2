using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShipGravityManager : MonoBehaviour
{
    //TODO:: generalize adding linear forces to and object etc.

    SortedSet<GravSOI> gravSOIs = new SortedSet<GravSOI>(new GravSOIComparer());
    public Action<Vector2> shipLinearDirectionInput;

    public Action<Vector2,float> shipGravitationalForce;


    private void OnTriggerEnter(Collider other)
    {
        GravSOI affectorOut;
        if (other.TryGetComponent<GravSOI>(out affectorOut))
        {
            if (!gravSOIs.Contains(affectorOut))
                gravSOIs.Add(affectorOut);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        GravSOI affectorOut;
        if (other.TryGetComponent<GravSOI>(out affectorOut))
        {
            if (gravSOIs.Contains(affectorOut))
                gravSOIs.Remove(affectorOut);
            else
                return;
        }
    }

    Vector2 m_Direction;
    float m_Force;
    void Update()
    {
        if (gravSOIs.Count <= 0)
            return;

        GravSOI gSOI = gravSOIs.Min;
        float distance = Vector3.Distance(transform.position, gSOI.transform.position);
        float mass = gSOI.mass;
        Vector3 direction = (gSOI.transform.position - transform.position).normalized;
        m_Force = mass / distance;
        m_Direction = direction;
        shipGravitationalForce(direction, mass / distance);
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.DrawSphere(transform.position, 0.01f);
        Gizmos.DrawRay(transform.position, m_Direction * m_Force);
    }
}

public class GravSOIComparer : IComparer<GravSOI>
{
    public int Compare(GravSOI x, GravSOI y)
    {
        if (x.mass > y.mass)
            return 1;
        else if (x.mass < y.mass)
            return -1;
        else
        {
            return 0;
        }
    }
}
